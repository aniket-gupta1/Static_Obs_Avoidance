import arcpy
import numpy

#Inputs from user parameters
InFc  = arcpy.GetParameterAsText(0) # input feature class
OutFc = arcpy.GetParameterAsText(1) # output feature class

# Spatial reference of input feature class
SR = arcpy.Describe(InFc).spatialReference

# Create NumPy array from input feature class
array = arcpy.da.FeatureClassToNumPyArray(InFc,["SHAPE@XY"], spatial_reference=SR, explode_to_points=True)

# Check array and Exit if no features found
if array.size == 0:
    arcpy.AddError(InFc + " has no features.")

# Create a new points feature class
else:
    arcpy.da.NumPyArrayToFeatureClass(array, OutFc, ['SHAPE@XY'], SR)
