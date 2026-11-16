// HPL adapter to the pinned, Apache-2.0 MuJoCo MPC SamplingPlanner.
#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/states/state.h"
#include "mjpc/threadpool.h"
#include "mjpc/planners/sampling/planner.h"

#ifdef _WIN32
#define LAB_API extern "C" __declspec(dllexport)
#else
#define LAB_API extern "C"
#endif
namespace {
struct References {
  int nq = 0, nv = 0, nbody = 0, rows = 0, stride = 0;
  double start = 0, dt = 0;
  std::vector<double> data;
  const double* at(double time) const {
    int index = std::clamp(static_cast<int>(std::llround((time-start)/dt)), 0, rows-1);
    return data.data() + index*stride;
  }
};
class TrackingTask final : public mjpc::Task {
 public:
  References refs;
  class TrackingResidual final : public mjpc::BaseResidualFn {
   public:
    const TrackingTask* owner;
    explicit TrackingResidual(const TrackingTask* task) : BaseResidualFn(task), owner(task) {}
    void Residual(const mjModel* m, const mjData* d, double* r) const override {
      if (owner->refs.rows == 0) { r[0] = 0; return; }
      const double* q = owner->refs.at(d->time);
      const double* v = q + m->nq;
      const double* p = v + m->nv;
      double cost = 0;
      for (int b=1; b<m->nbody; ++b) {
        if (m->body_mass[b] < .01 || m->body_rootid[b]!=1) continue;  // massless hinge carriers
        for (int k=0; k<3; ++k) {
          double e = (d->xpos[3*b+k]-p[3*b+k])/.10;
          cost += e*e;
        }
      }
      for (int j=7; j<m->nq; ++j) {
        double e = (d->qpos[j]-q[j])/.5;
        cost += .2*e*e;
      }
      for (int j=0; j<3; ++j) {
        double e = d->qvel[j]-v[j];
        cost += 2*e*e;
      }
      double dot = 0;
      for (int j=3; j<7; ++j) dot += d->qpos[j]*q[j];
      cost += 20*(1-dot*dot);
      r[0] = cost;
    }
  };
  TrackingTask() {
    num_residual=1; num_term=1; num_trace=0; risk=0; mode=0;
    dim_norm_residual={1}; num_norm_parameter={0};
    norm={static_cast<mjpc::NormType>(-1)}; weight={1};
    residual_=std::make_unique<TrackingResidual>(this);
  }
  std::string Name() const override { return "HPL physical reference tracking"; }
  std::string XmlPath() const override { return ""; }
 protected:
  mjpc::BaseResidualFn* InternalResidual() override { return residual_.get(); }
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<TrackingResidual>(this);
  }
 private:
  std::unique_ptr<TrackingResidual> residual_;
};
struct Context {
  mjModel* model;
  TrackingTask task;
  mjpc::SamplingPlanner planner;
  mjpc::State state;
  mjpc::ThreadPool pool{2};
  int horizon;
  Context(const mjModel* source, int samples, int horizon_) : horizon(horizon_) {
    model=mj_copyModel(nullptr, source);
    // Prediction is approximate; execution retains the shared 960 Hz model.
    model->opt.timestep=1.0/120;
    for (int a=0; a<model->nu; ++a) {
      int j=model->actuator_trnid[2*a];
      model->actuator_ctrlrange[2*a]=model->jnt_range[2*j];
      model->actuator_ctrlrange[2*a+1]=model->jnt_range[2*j+1];
    }
    planner.Initialize(model, task);
    planner.Allocate();
    planner.num_trajectory_=samples;
    planner.noise_exploration[0]=.025;
    planner.noise_exploration[1]=0;
    planner.policy.num_spline_points=5;
    planner.Reset(horizon);
    state.Allocate(model);
  }
  ~Context() { mj_deleteModel(model); }
};
Context* active=nullptr;
thread_local std::string last_error;
void Sensor(const mjModel* m, mjData* d, int stage) {
  if (active && m==active->model && stage==mjSTAGE_ACC)
    active->task.Residual(m,d,d->sensordata);
}
}
LAB_API const char* lab_error() { return last_error.c_str(); }
LAB_API int lab_version() { return mj_version(); }
LAB_API void* lab_create(const mjModel* model, int samples, int horizon) {
  try {
    if (active) throw std::runtime_error("Only one active planner is supported");
    if (model->nsensordata!=1) throw std::runtime_error("Expected one inert residual sensor");
    if (samples<2 || samples>128 || horizon<2 || horizon>512)
      throw std::runtime_error("Invalid planner dimensions");
    auto* context=new Context(model,samples,horizon);
    active=context;
    mjcb_sensor=Sensor;
    return context;
  } catch(const std::exception& e) { last_error=e.what(); return nullptr; }
}
LAB_API int lab_plan(void* handle, const mjData* current, const double* refs,
                    int rows, double reference_dt, double* control) {
  try {
    auto* c=static_cast<Context*>(handle);
    auto& r=c->task.refs;
    r.nq=c->model->nq; r.nv=c->model->nv; r.nbody=c->model->nbody;
    References previous=r;
    r.rows=rows; r.dt=reference_dt; r.start=current->time;
    r.stride=r.nq+r.nv+3*r.nbody;
    r.data.assign(refs,refs+rows*r.stride);
    // Seed with the reference joint targets, not a neural-policy action.
    auto& plan=c->planner.policy.plan;
    std::vector<double> warm_start(5*c->model->nu,0);
    if (previous.rows) {
      for (int k=0;k<5;++k) {
        double time=current->time+k*(c->horizon-1)*c->model->opt.timestep/4;
        c->planner.ActionFromPolicy(warm_start.data()+k*c->model->nu,nullptr,time);
        const double* prior=previous.at(time);
        for (int a=0;a<c->model->nu;++a) {
          int j=c->model->actuator_trnid[2*a];
          warm_start[k*c->model->nu+a]-=prior[c->model->jnt_qposadr[j]];
        }
      }
    }
    plan.Clear();
    for (int k=0;k<5;++k) {
      double time=current->time+k*(c->horizon-1)*c->model->opt.timestep/4;
      const double* q=r.at(time);
      std::vector<double> target(c->model->nu);
      for(int a=0;a<c->model->nu;++a) {
        int joint=c->model->actuator_trnid[2*a];
        target[a]=q[c->model->jnt_qposadr[joint]]+warm_start[k*c->model->nu+a];
      }
      plan.AddNode(time,absl::MakeConstSpan(target.data(),target.size()));
    }
    c->state.Set(c->model,current);
    c->planner.SetState(c->state);
    c->planner.OptimizePolicy(c->horizon,c->pool);
    c->planner.ActionFromPolicy(control,nullptr,current->time);
    return 1;
  } catch(const std::exception& e) { last_error=e.what(); return 0; }
}
LAB_API void lab_destroy(void* handle) {
  mjcb_sensor=nullptr;
  active=nullptr;
  delete static_cast<Context*>(handle);
}
