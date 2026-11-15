/** Namespaces the actor subtree produced by our synthetic MJCF generator.
 * World geoms/props are deliberately outside that subtree and keep their names.
 */
export function extractBodyTree(xml: string, rootName: string): {
  start: number;
  end: number;
  xml: string;
} {
  const marker = `<body name="${rootName}"`;
  const start = xml.indexOf(marker);
  if (start < 0)
    throw new Error(`No existe el cuerpo raíz ${rootName}`);
  const tags = /<body\b[^>]*>|<\/body>/g;
  tags.lastIndex = start;
  let depth = 0;
  for (let m = tags.exec(xml); m; m = tags.exec(xml)) {
    depth += m[0].startsWith("</") ? -1 : 1;
    if (depth === 0)
      return { start, end: tags.lastIndex, xml: xml.slice(start, tags.lastIndex) };
  }
  throw new Error(`Jerarquía incompleta de ${rootName}`);
}
export function namespaceHumanoidXML(xml: string, prefix: string): string {
  if (!/^(?:[a-z][a-z0-9_]*_)?$/.test(prefix))
    throw new RangeError("Prefijo de humanoide inválido");
  if (!prefix)
    return xml;
  const body = extractBodyTree(xml, "pelvis");
  const renamed = body.xml.replace(/\bname="([^"]+)"/g, (_all, name: string) => `name="${prefix}${name}"`);
  let out = xml.slice(0, body.start) + renamed + xml.slice(body.end);
  out = out.replace(/<contact>([\s\S]*?)<\/contact>/g, (_all, content: string) => `<contact>${content.replace(/\b(body1|body2)="([^"]+)"/g, (_a, key: string, name: string) => `${key}="${prefix}${name}"`)}</contact>`);
  return out;
}
