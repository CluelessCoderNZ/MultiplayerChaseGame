extends Node2D

var points = 12
var radius = 50.0
var circumfrenceMultiplier = 1.0

var area = radius * radius * PI
var circumfrence = radius * 2.0 * PI * circumfrenceMultiplier
var length = circumfrence * 1.15 / float(points)
var iterations = 10


export(float) var default_radius = 50.0
export(float) var default_circumfrence_multiplier = 1.0
export(int)   var default_points = 12

export var physics_material: PhysicsMaterial
export(float) var collision_radius = 5
export(float) var jump_strength = 1000
export(float) var gravity = 1
export(float) var linear_damp = 0.1
export(float) var shape_impulse_strength = 5

export(Color) var outline
export(Color) var fill

var blob_colliders = []
var blob = []
var blobOld = []
var accumulatedDisplacements = {}
var normals = []
var center = Vector2(0.0, 0.0)
export(float) var splineLength = 12.0
export(Curve2D) var curve

func verletIntegrate(i, delta):
	var temp = Vector2(blob[i])
	blob[i] = (blob[i] + (blob[i] - blobOld[i]))
	blobOld[i] = temp

func setDistance(currentPoint, anchor, distance):
	var toAnchor = currentPoint - anchor
	toAnchor = toAnchor.normalized() * distance
	return toAnchor + anchor

func _ready():
	set_circumfrence(default_circumfrence_multiplier)
	set_radius(default_radius)
	set_point_count(default_points)

func findCentroid():
	var x = 0.0
	var y = 0.0
	for i in range(points): 
		x += blob[i].x
		y += blob[i].y
	var cent = Vector2(0.0, 0.0)
	cent.x = x / float(points)
	cent.y = y / float(points)
	return cent

func getPoint(i):
	var pointCount = curve.get_point_count()
	i = wrapi(i, 0, pointCount - 1)
	return curve.get_point_position(i)

func getSpline(i):
	var lastPoint = getPoint(i - 1)
	var nextPoint = getPoint(i + 1)
	var spline = lastPoint.direction_to(nextPoint) * splineLength
	return spline

func updateSprite ():
	var polBlob = blob + [blob[0]]
	
	curve.clear_points()
	for i in range(points + 1):
		curve.add_point(polBlob[i] - position)
	
	var point_count = curve.get_point_count()
	for i in point_count:
		var spline = getSpline(i)
		curve.set_point_in(i, -spline)
		curve.set_point_out(i, spline)


func getCurArea ():
	var area = 0.0
	var j = points - 1 
	for i in range(points):
		area += (blob[j].x + blob[i].x) * (blob[j].y - blob[i].y)
		j = i
	return abs(area / 2.0)

func getClosestPointArg(contact: Vector2):
	var min_val = INF
	var min_idx = null
	for i in range(points):
		var dist = contact.distance_squared_to(blob[i])
		if min_val > dist:
			min_val = dist
			min_idx = i
	
	return min_idx
	
func getVectorByLA (length, angle):
	angle = deg2rad(angle)
	var vector = Vector2(cos(angle), sin(angle)) * length
	return vector

func _draw():
	
	var bakedPoints = Array(curve.get_baked_points())
	
	var drawPoints = bakedPoints + []
	
	if Geometry.triangulate_polygon(drawPoints).empty():
		drawPoints = Geometry.convex_hull_2d(bakedPoints)
	
	
	draw_polygon(drawPoints, [fill])
	
	draw_polyline(bakedPoints, outline, 5.0, true)
	
#	for i in range (points):
#		draw_line(blob[i], blob[(i + 1) % points], Color.blue, 3)
		
	
func _physics_process(delta):
	for i in range(blob_colliders.size()):
		blob[i] = blob_colliders[i].position
	
	for i in range(points):
		verletIntegrate(i, delta)
	
	for iterate in range(iterations):
		for i in range(points):
			var segment = blob[i]
			var nextIndex = i + 1
			if i == points - 1:
				nextIndex = 0
			var nextSegment = blob[nextIndex]
			var toNext = segment - nextSegment
			if toNext.length() > length:
				toNext = toNext.normalized() * length
				var offset = (segment - nextSegment) - toNext
				accumulatedDisplacements[(i * 3)] -= offset.x / 2.0
				accumulatedDisplacements[(i * 3) + 1] -= offset.y / 2.0
				accumulatedDisplacements[(i * 3) + 2] += 1.0
				accumulatedDisplacements[(nextIndex * 3)] += offset.x / 2.0
				accumulatedDisplacements[(nextIndex * 3) + 1] += offset.y / 2.0
				accumulatedDisplacements[(nextIndex * 3) + 2] += 1.0
			
		var deltaArea = 0.0
		var curArea = getCurArea()
		if curArea < area * 2.0:
			deltaArea = area - curArea
		var dilationDistance = deltaArea / circumfrence
	
		for i in range(points):
			var prevIndex = i - 1
			if i == 0:
				prevIndex = points - 1
			var nextIndex = i + 1
			if i == points - 1:
				nextIndex = 0
			var normal = blob[nextIndex] - blob[prevIndex]
			normal = getVectorByLA(1, rad2deg(normal.angle()) - 90.0)
			normals[i][0] = blob[i]
			normals[i][1] = blob[i] + (normal * 200.0)
			accumulatedDisplacements[(i * 3)] += normal.x * dilationDistance
			accumulatedDisplacements[(i * 3) + 1] += normal.y * dilationDistance
			accumulatedDisplacements[(i * 3) + 2] += 1.0
	
		for i in range (points):
			if (accumulatedDisplacements[(i * 3) + 2] > 0):
				 blob_colliders[i].apply_central_impulse(shape_impulse_strength * Vector2(accumulatedDisplacements[(i * 3)], accumulatedDisplacements[(i * 3) + 1]) / accumulatedDisplacements[(i * 3) + 2])
	
		for i in range (points * 3): 
			accumulatedDisplacements[i] = 0
	
	position = findCentroid()
	
	updateSprite()
	update()

func _process(delta):
	if Input.is_action_just_pressed("ui_accept"):
		var x = Input.get_joy_axis(0, JOY_AXIS_0)
		var y = Input.get_joy_axis(0, JOY_AXIS_1)
		for i in range(blob_colliders.size()):
			blob_colliders[i].apply_central_impulse(Vector2(x,y)*jump_strength)




func create_collider(pos, radius):
	var collider = RigidBody2D.new()
	var collider_shape = CollisionShape2D.new()
	var circle_shape = CircleShape2D.new()
	circle_shape.radius = radius
	collider_shape.set_shape(circle_shape)
	collider.add_child(collider_shape)
	collider.collision_mask = 0
	collider.physics_material_override = physics_material
	collider.position = pos
	collider.gravity_scale = gravity
	collider.linear_damp = linear_damp
	collider.continuous_cd = RigidBody2D.CCD_MODE_CAST_RAY
	collider.set_as_toplevel(true)
	self.add_child(collider)
	return collider

func resetBlob ():
	for n in self.get_children():
		self.remove_child(n)
		n.queue_free()
	
	blob = []
	blobOld = []
	normals = []
	blob_colliders = []
	accumulatedDisplacements = {}
	for i in range(points):
		var delta = getVectorByLA(radius, (360.0 / float(points)) * i)
		blob.append(position + delta)
		blobOld.append(position + delta)
		blob_colliders.append(create_collider(position + delta, collision_radius))
		normals.append([])
		normals[i].append(position + delta)
		normals[i].append(position + delta * 1.5)
	for i in range (points * 3): 
		accumulatedDisplacements[i] = 0.0
	updateSprite()
	update()


# Circumfrence Slider
func set_circumfrence(value):
	circumfrenceMultiplier = value
	circumfrence = radius * 2.0 * PI * circumfrenceMultiplier
	length = circumfrence * 1.15 / float(points)

# Area Slider
func set_radius(value):
	radius = value
	area = radius * radius * PI
	circumfrence = radius * 2.0 * PI * circumfrenceMultiplier
	length = circumfrence * 1.15 / float(points)

# Points Slider
func set_point_count(value):
	points = value
	area = radius * radius * PI
	circumfrence = radius * 2.0 * PI * circumfrenceMultiplier
	length = circumfrence * 1.15 / float(points)
	
	resetBlob()
