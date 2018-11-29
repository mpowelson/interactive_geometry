import numpy as np
from stl import mesh

class MeshExporter:
    def __init__(self):
        print("")

    def mesh_to_stl(self, vertices, faces, filename):
        print("Saving mesh as stl")
        # Create the mesh
        output = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
        for i, f in enumerate(faces.astype(int)):
            for j in range(3):
                output.vectors[i][j] = vertices[f[j],:]

        # Write the mesh to file
        output.save(filename)
        return True
