#ifndef STEREOAPPPLUGININTERFACE_H
#define STEREOAPPPLUGININTERFACE_H

#include <QtPlugin>

#define StereoVisionApp_PluginInterface_iid "org.StereoVisionApp.PluginInterface"

namespace StereoVisionApp {

class StereoVisionApplication;

class StereoAppPluginInterface
{
public:
    StereoAppPluginInterface();
    virtual ~StereoAppPluginInterface() {}

    virtual int loadModule(StereoVisionApplication* app) const = 0;
    virtual int cleanupModule(StereoVisionApplication* app) const = 0;
};

}

Q_DECLARE_INTERFACE(StereoVisionApp::StereoAppPluginInterface, StereoVisionApp_PluginInterface_iid)

#endif // STEREOAPPPLUGININTERFACE_H
