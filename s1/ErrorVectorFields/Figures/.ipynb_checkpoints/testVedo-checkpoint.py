import vtk
vtk.vtkObject.GlobalWarningDisplayOn()

from vedo import Plotter, Cube

cube = Cube()
plotter = Plotter()
plotter.show(cube, interactive=True)