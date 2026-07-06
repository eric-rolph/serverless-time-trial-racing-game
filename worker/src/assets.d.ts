// Wrangler bundling rules (wrangler.toml): CompiledWasm and Data imports.
declare module "*.wasm" {
  const module: WebAssembly.Module;
  export default module;
}
declare module "*.bin" {
  const data: ArrayBuffer;
  export default data;
}
