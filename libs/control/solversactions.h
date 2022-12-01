#ifndef SOLVERSACTIONS_H
#define SOLVERSACTIONS_H

namespace StereoVisionApp {

class Project;
class MainWindow;

bool resetSolution(Project* p, MainWindow* w = nullptr);
void solveCoarse(Project* p, MainWindow* w = nullptr, int nStep = 500);
void initSolution(Project* p, MainWindow* w = nullptr);
void initMonoStereoRigSolution(Project* p, MainWindow* w = nullptr);
void solveSparse(Project* p, MainWindow* w = nullptr, int nStep = 50);
void solveSparseStereoRig(Project* p, MainWindow* w = nullptr, int nStep = 50);

} //namespace StereoVisionApp

#endif // SOLVERSACTIONS_H
