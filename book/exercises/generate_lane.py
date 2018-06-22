
m_from_in = lambda x: x * 0.0254


tile = m_from_in(12*2)

WW = m_from_in(2)
WY = m_from_in(1)
# L = (tile - 2*WW - 2*WY) / 2
L = (tile - 2*WW - WY) / 2


y1 = + L + WW
y2 = + L
y3 = + WY/2
y4 = - WY/2
y5 = - L
y6 = - L - WW
D = 2 * tile

yaml = """
# lane width (inner white from inner yellow): L
points:
     p1: [axle, [0, y1, 0]]
     q1: [axle, [D, y1, 0]]
     p2: [axle, [0, y2, 0]]
     q2: [axle, [D, y2, 0]]
     p3: [axle, [0, y3, 0]]
     q3: [axle, [D, y3, 0]]
     p4: [axle, [0, y4, 0]]
     q4: [axle, [D, y4, 0]]
     p5: [axle, [0, y5, 0]]
     q5: [axle, [D, y5, 0]]
     p6: [axle, [0, y6, 0]]
     q6: [axle, [D, y6, 0]]
segments:
 - points: [p1, q1]
   color: white
 - points: [p2, q2]
   color: white
 - points: [p3, q3]
   color: yellow
 - points: [p4, q4]
   color: yellow
 - points: [p5, q5]
   color: white
 - points: [p6, q6]
   color: white
"""

V = dict(y1=y1,y2=y2,y3=y3,y4=y4,y5=y5,y6=y6, L=L, D=D)
for k,v in V.items():
    yaml = yaml.replace(k, str(v))

print yaml
