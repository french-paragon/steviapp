#ifndef STEREOVISIONAPP_DATATABLEACTIONS_H
#define STEREOVISIONAPP_DATATABLEACTIONS_H


namespace StereoVisionApp {

class Project;
class DataBlock;

bool openDataTableFromCsv(Project* p);

bool openDataTable(DataBlock* d);

bool removeDataTable(DataBlock* d);

bool exportDataTableToCsv(DataBlock* d);

} // namespace StereoVisionApp

#endif // STEREOVISIONAPP_DATATABLEACTIONS_H
