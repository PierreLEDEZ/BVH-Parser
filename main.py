from src import bvh_parser

# Create the BVH parser
bvhParser = bvh_parser.BVHParser()

# Parse the given file
bvhParser.parse("./examples/scene.bvh")

# Obtain informations about bvh file
bvhParser.get_informations()

# Plot hierarchy, this is the skeleton in the T-pose
bvhParser.plot_hierarchy()

# Print the hierarchy
bvhParser.print_hierarchy()