import base64, sys
tpl, wasm, out = sys.argv[1:4]
html = open(tpl, encoding="utf-8").read()
b64 = base64.b64encode(open(wasm, "rb").read()).decode()
open(out, "w", encoding="utf-8").write(html.replace("__WASM_BASE64__", b64))
