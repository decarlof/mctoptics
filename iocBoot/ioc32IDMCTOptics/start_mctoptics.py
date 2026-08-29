# This script creates an object of type MCTOptics for doing tomography streaming reconstruction
# To run this script type the following:
#     python -i start_mctoptics.py
# The -i is needed to keep Python running, otherwise it will create the object and exit
from mctoptics.mctoptics import MCTOptics
ts = MCTOptics(["../../db/mctOptics_settings.req", ], {"$(P)":"32id:", "$(R)":"MCTOptics:"})
