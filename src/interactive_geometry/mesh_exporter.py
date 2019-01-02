"""
Contains the MeshExporter class for exporting meshes to files
"""

from numpy import zeros
from stl import mesh

class MeshExporter:
    """Contains various functions for exporting meshes to mesh file formats

    You can currently choose any format known to man - as long as its stl
    """
    def __init__(self):
        print("")

    def mesh_to_stl(self, vertices, faces, filename):
        """
        Exports a set of vertices and faces as an stl to the given filename using the stl library
        """
        print("Saving mesh as stl")
        # Create the mesh
        output = mesh.Mesh(zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
        for i, f in enumerate(faces.astype(int)):
            for j in range(3):
                output.vectors[i][j] = vertices[f[j],:]

        # Write the mesh to file
        output.save(filename)
        return True
