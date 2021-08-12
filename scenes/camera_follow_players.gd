extends Camera2D


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.

func _process(delta):
	var player_centroid = Vector2(0,0)
	var player_nodes = get_tree().get_nodes_in_group("player_object")
	for player_node in player_nodes:
		player_centroid += player_node.position
	position = player_centroid / len(player_nodes)
