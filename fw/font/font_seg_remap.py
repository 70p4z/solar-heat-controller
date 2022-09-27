import os
import sys
import traceback

font_remap = sys.argv[1].upper()

# base_font segment map
segline=""
for line in sys.stdin:
	if not line.startswith("#"): #skip comments
		segline = line.strip()
		break
font_base_map = segline

print ("{")
print ("//" + font_base_map)
print ("//" + font_remap)

# compute remapping move to remap the base font
fetch_pos = []
for c in font_remap:
	d=0
	while d<len(font_base_map):
		if font_base_map[d] == c:
			fetch_pos.append(d)
			break
		d+=1
	if d == len(font_base_map):
		raise BaseException(f"Can't find segment {c} in font base map")

# read all line
for charsegs in sys.stdin:
	try:
		if not charsegs.startswith("#"): #skip comments
			remapcharsegs = ""
			for pos in fetch_pos:
				remapcharsegs += charsegs[pos]
			#output remapped segments
			print("0b"+remapcharsegs+",")
	except:
		traceback.print_exc()
print("};")
