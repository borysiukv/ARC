"""Generate a PROGMEM header from data/index.html. Python stdlib only."""
from pathlib import Path
root = Path(__file__).resolve().parent.parent
html = (root/'data'/'index.html').read_text(encoding='utf-8')
assert ')ARC26HTML"' not in html
header = '#pragma once\n#include <Arduino.h>\nstatic const char ARC_HTML[] PROGMEM = R"ARC26HTML(' + html + ')ARC26HTML";\n'
(root/'include'/'dashboard_html.h').write_text(header,encoding='utf-8')
print(f'Embedded {len(html.encode("utf-8"))} bytes')
