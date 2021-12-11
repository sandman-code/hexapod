from pygcode import Line

with open('test.gcode', 'r') as fh:
    for line_text in fh.readlines():
        line = Line(line_text)
        print(line)
