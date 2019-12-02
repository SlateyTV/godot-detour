extends Spatial

const DetourNavigation 	            :NativeScript = preload("res://addons/godotdetour/detournavigation.gdns")
const DetourNavigationParameters	:NativeScript = preload("res://addons/godotdetour/detournavigationparameters.gdns")
const DetourNavigationMeshParameters    :NativeScript = preload("res://addons/godotdetour/detournavigationmeshparameters.gdns")
const DetourCrowdAgent	            :NativeScript = preload("res://addons/godotdetour/detourcrowdagent.gdns")
const DetourCrowdAgentParameters    :NativeScript = preload("res://addons/godotdetour/detourcrowdagentparameters.gdns")
const DetourObstacle				:NativeScript = preload("res://addons/godotdetour/detourobstacle.gdns")
const CustomArray					:NativeScript = preload("res://addons/godotdetour/customarray.gdns")


var navigation = null
var testIndex :int = -1
onready var nextStepLbl : RichTextLabel = get_node("Control/NextStepLbl")
var debugMeshInstance :MeshInstance = null

var levelStaticBody			:StaticBody = null
var doPlaceRemoveObstacle 	:bool = false
var doMarkArea				:bool = false
var rayQueryPos				:Vector3 = Vector3(0, 0, 0)
var obstacles				:Dictionary = {}

# Called when the node enters the scene tree for the first time.
func _ready():
	# Create the detour navigation
	pass
	
# Called when the user presses a key
func _input(event :InputEvent) -> void:
	# Quit the application
	if event.is_action("ui_cancel") && event.is_pressed():
		get_tree().quit()
	# Start the next test
	if event.is_action("ui_select") && event.is_pressed():
		testIndex += 1
		doNextTest(testIndex)
	# Place/remove obstacle
	if testIndex == 1 && event.is_action("obstacle") && event.is_pressed():
		rayQueryPos = $Camera.translation
		doPlaceRemoveObstacle = true
	# Mark area
	if testIndex == 1 && event.is_action("mark_area") && event.is_pressed():
		rayQueryPos = $Camera.translation
		doMarkArea = true

# Do the next test in line
func doNextTest(index :int) -> void:
	if index == 0:
		nextStepLbl.text = "Initializing the navigation..."
		yield(get_tree(), "idle_frame")
		initializeNavigation()
		nextStepLbl.text = "Next step:      Enable Navigation Debug Drawing"
	if index == 1:
		nextStepLbl.text = "Drawing debug mesh..."
		yield(get_tree(), "idle_frame")
		drawDebugMesh()
		nextStepLbl.visible = false;
		$Control/TopLbl.bbcode_text = "[b](LMB)[/b] place/remove agent [b](RMB)[/b] set destination [b](F)[/b] place/remove obstacle [b](M)[/b] mark water area"

# Initializes the navigation
func initializeNavigation():
	# Create the navigation parameters
	var navParams = DetourNavigationParameters.new()
	navParams.ticksPerSecond = 60 # How often the navigation is updated per second in its own thread
	navParams.maxObstacles = 256 # How many dynamic obstacles can be present at the same time
	
	# Create the parameters for the "small" navmesh
	var navMeshParamsSmall = DetourNavigationMeshParameters.new()
	# It is important to understand that recast/detour operates on a voxel field internally.
	# The size of a single voxel (often called cell internally) has significant influence on how a navigation mesh is created.
	# A tile is a rectangular region within the navigation mesh. In other words, every navmesh is divided into equal-sized tiles, which are in turn divided into cells.
	# The detail mesh is a mesh used for determining surface height on the polygons of the navigation mesh.
	# Units are usually in world units [wu] (e.g. meters, or whatever you use), but some may be in voxel units [vx] (multiples of cellSize).
	
	# x = width & depth of a single cell (only one value as both must be the same) | y = height of a single cell. [wu]
	navMeshParamsSmall.cellSize = Vector2(0.15, 0.1)
	# How steep an angle can be to still be considered walkable. In degree. Max 90.0.
	navMeshParamsSmall.maxAgentSlope = 45.0
	# The maximum height of an agent supported in this navigation mesh. [wu]
	navMeshParamsSmall.maxAgentHeight = 2.0
	# How high a single "stair" can be to be considered walkable by an agent. [wu]
	navMeshParamsSmall.maxAgentClimb = 1.0
	# The maximum radius of an agent in this navigation mesh. [wu]
	navMeshParamsSmall.maxAgentRadius = 0.5
	# The maximum allowed length for contour edges along the border of the mesh. [wu]
	navMeshParamsSmall.maxEdgeLength = 2.0
	# The maximum distance a simplified contour's border edges should deviate the original raw contour. [vx]
	navMeshParamsSmall.maxSimplificationError = 1.3
	# How many cells an isolated area must at least have to become part of the navmesh.
	navMeshParamsSmall.minNumCellsPerIsland = 8
	# Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
	navMeshParamsSmall.minCellSpanCount = 20
	# Maximum number of vertices per polygon in the navigation mesh.
	navMeshParamsSmall.maxVertsPerPoly = 6
	# The width,depth & height of a single tile. [vx]
	navMeshParamsSmall.tileSize = 42
	# How many vertical layers a single tile is expected to have. Should be less for "flat" levels, more for something like tall, multi-floored buildings.
	navMeshParamsSmall.layersPerTile = 4
	# The sampling distance to use when generating the detail mesh. [wu]
	navMeshParamsSmall.detailSampleDistance = 6.0
	# The maximum allowed distance the detail mesh should deviate from the source data. [wu]
	navMeshParamsSmall.detailSampleMaxError = 1.0
	navParams.navMeshParameters.append(navMeshParamsSmall)
	
	# Create the parameters for the "large" navmesh
	var navMeshParamsLarge = DetourNavigationMeshParameters.new()
	navMeshParamsLarge.cellSize = Vector2(0.5, 0.35)
	navMeshParamsLarge.maxAgentSlope = 45.0
	navMeshParamsLarge.maxAgentHeight = 4.0
	navMeshParamsLarge.maxAgentClimb = 0.5
	navMeshParamsLarge.maxAgentRadius = 2.5
	navMeshParamsLarge.maxEdgeLength = 12.0
	navMeshParamsLarge.maxSimplificationError = 1.3
	navMeshParamsLarge.minNumCellsPerIsland = 8
	navMeshParamsLarge.minCellSpanCount = 20
	navMeshParamsLarge.maxVertsPerPoly = 6
	navMeshParamsLarge.tileSize = 42
	navMeshParamsLarge.layersPerTile = 4
	navMeshParamsLarge.detailSampleDistance = 6.0
	navMeshParamsLarge.detailSampleMaxError = 1.0
	navParams.navMeshParameters.append(navMeshParamsLarge)
	
	# Create the arrayMesh from the CSG and set it as the meshInstance's mesh
	var csgCombiner :CSGShape = get_node("CSGCombiner")
	csgCombiner._update_shape()
	var arrayMesh :ArrayMesh = csgCombiner.get_meshes()[1]
	var meshInstance :MeshInstance = get_node("MeshInstance")
	meshInstance.mesh = arrayMesh
	meshInstance.create_trimesh_collision()
	levelStaticBody = meshInstance.get_child(0)
	remove_child(csgCombiner)
	
	# Mark an area in the center as grass, this is doable before initalization
	navigation = DetourNavigation.new()
	var vertices = CustomArray.new() 
	vertices.append(Vector3(-2.0, -0.5, 1.7))
	vertices.append(Vector3(3.2, -0.5, 2.2))
	vertices.append(Vector3(2.3, -0.5, -2.0))
	vertices.append(Vector3(-1.2, -0.5, -3.1))
	var markedAreaId = navigation.markConvexArea(vertices, 1.5, 4) # 4 = grass
	
	# Initialize the navigation with the mesh instance and the parameters
	navigation.initialize(meshInstance, navParams)

# Draws and displays the debug mesh
func drawDebugMesh() -> void:
	# Free the old instance
	if debugMeshInstance != null:
		remove_child(debugMeshInstance)
		debugMeshInstance.queue_free()
		debugMeshInstance = null
	
	# Create the debug mesh
	debugMeshInstance = navigation.createDebugMesh(0, false)
	if !debugMeshInstance:
		printerr("Debug meshInst invalid!")
		return
	
	# Add the debug mesh instance a little elevated to avoid flickering
	debugMeshInstance.translation = Vector3(0.0, 0.05, 0.0)
	var displayMeshInst :MeshInstance = get_node("MeshInstance")
	debugMeshInstance.rotation = displayMeshInst.rotation
	add_child(debugMeshInstance)
	
# Place or remove an obstacle
func placeRemoveObstacle() -> void:
	# Do a ray query
	pass

# Called during physics process updates (doing creation/removal of obstacles and agents, etc.)
func _physics_process(delta):
	if doPlaceRemoveObstacle == true or doMarkArea == true:
		var redrawDebugMesh :bool = false
		
		# Adjust the collision mask
		var collisionMask = 1
		if doPlaceRemoveObstacle:
			collisionMask = 1 | 2
		
		# Querying is the same for obstacles, marks & agents
		var cam :Camera = $Camera
		var to :Vector3 = rayQueryPos + 1000.0 * -cam.global_transform.basis.z
		var spaceState :PhysicsDirectSpaceState = get_world().direct_space_state
		var result :Dictionary = spaceState.intersect_ray(rayQueryPos, to, [], collisionMask)
		
		# Quit if we didn't hit anything
		if result.empty():
			return
		
		# Place or remove an obstacle
		if doPlaceRemoveObstacle == true:
			doPlaceRemoveObstacle = false
			redrawDebugMesh = true
			
			# Check if we hit the level geometry
			if result.collider == levelStaticBody:
				# Create an obstacle in Godot
				var newObstacle :RigidBody = $Obstacle.duplicate()
				newObstacle.translation = result.position
				add_child(newObstacle)
				
				# Create an obstacle in GodotDetour and remember both
				var targetPos :Vector3 = result.position
				targetPos.y -= 0.2
				var godotDetourObstacle = navigation.addCylinderObstacle(targetPos, 0.7, 2.0)
				obstacles[newObstacle] = godotDetourObstacle
			# Otherwise, we hit an obstacle
			else:
				# Remove the obstacle
				var obstacle :RigidBody = result.collider
				var godotDetourObstacle = obstacles[obstacle]
				godotDetourObstacle.destroy() # This is important! Don't leave memory leaks
				obstacles.erase(obstacle)
				remove_child(obstacle)
				obstacle.queue_free()
		
		# Mark a somewhat random area
		if doMarkArea == true:
			doMarkArea = false
			redrawDebugMesh = true
			
			var vertices = CustomArray.new() 
			var targetPos :Vector3 = result.position
			vertices.append(targetPos + Vector3(rand_range(-0.5, -2.0), -0.5, rand_range(-0.5, -2.0)))
			vertices.append(targetPos + Vector3(rand_range(0.5, 2.0), -0.5, rand_range(-0.5, -2.0)))
			vertices.append(targetPos + Vector3(rand_range(0.5, 2.0), -0.5, rand_range(0.5, 2.0)))
			vertices.append(targetPos + Vector3(rand_range(-0.5, -2.0), -0.5, rand_range(0.5, 2.0)))
			var markedAreaId = navigation.markConvexArea(vertices, 1.5, 2) # 2 = water
			
			# Doing this right after marking a single area is not good for performance
			# It is just done this way here for demo purposes
			navigation.rebuildChangedTiles()
			
		# Update the debug mesh after a bit (letting the navigation thread catch up)
		if redrawDebugMesh == true:
			var timer :Timer = Timer.new()
			timer.set_one_shot(true)
			timer.set_wait_time(0.1)
			timer.connect("timeout", self, "drawDebugMesh")
			add_child(timer)
			timer.start()
