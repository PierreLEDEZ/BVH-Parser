from src import bvh_parser

# Create the BVH parser
bvhParser = bvh_parser.BVHParser()

# Parse the given file
bvhParser.parse("./examples/scene_2.bvh")

# Delete children of RightHand and LeftHand
# By default 2nd option "inclusive" is False so in this case, kept joints named RightHand and LeftHand
bvhParser.delete_joint("RightHand")
bvhParser.delete_joint("LeftHand")

# Obtain informations about bvh file
bvhParser.get_informations()

# Plot hierarchy, this is the skeleton in the T-pose
bvhParser.plot_hierarchy()

# Print the hierarchy
bvhParser.print_hierarchy()