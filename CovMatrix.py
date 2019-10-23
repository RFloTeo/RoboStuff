points = [
  (-5.5, 5), 
  (2.1, 3.1),
  (-1.6, 2.5),
  (-1.1, 2.4),
  (0.5, 3.2),
  (-0.6, 0.9),
  (-0.5, 0),
  (2, -1.3),
  (2.9, -1.2),
  (3.8, -1)
]

N = len(points)
Xmean = sum(map(lambda point: point[0], points)) / N
Ymean = sum(map(lambda point: point[1], points)) / N

output = [[0,0],[0,0]]

output[0][0] = sum(map(lambda point: (point[0] - Xmean) ** 2, points)) / N
output[0][1] = sum(map(lambda point: (point[0] - Xmean) * (point[1] - Ymean), points)) / N
output[1][0] = sum(map(lambda point: (point[1] - Ymean) * (point[0] - Xmean), points)) / N
output[1][1] = sum(map(lambda point: (point[1] - Ymean) ** 2, points)) / N

print(output[0])
print(output[1])
