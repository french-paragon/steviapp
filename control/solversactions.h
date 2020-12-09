#ifndef SOLVERSACTIONS_H
#define SOLVERSACTIONS_H

namespace StereoVisionApp {

class Project;
class MainWindow;

bool resetSolution(Project* p, MainWindow* w = nullptr);
void initSolution(Project* p, MainWindow* w = nullptr);
void solveSparse(Project* p, MainWindow* w = nullptr, int nStep = 50);

} //namespace StereoVisionApp

#endif // SOLVERSACTIONS_H
