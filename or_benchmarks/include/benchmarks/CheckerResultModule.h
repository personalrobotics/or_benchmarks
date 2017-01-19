#ifndef CHECKER_RESULT_MODULE_H
#define CHECKER_RESULT_MODULE_H

#include <openrave/openrave.h>
#include <benchmarks/picojson.h>

namespace benchmarks {

class CheckerResultModule : public OpenRAVE::ModuleBase {

public:
    CheckerResultModule(OpenRAVE::EnvironmentBasePtr env);

    virtual ~CheckerResultModule();

private:

    bool EvaluateCheck(std::ostream &out, std::istream &in);
    OpenRAVE::KinBodyConstPtr DeserializeKinBody(std::string keyname, picojson::value val);
    OpenRAVE::KinBody::LinkConstPtr DeserializeLink(std::string keyname, picojson::value val);
    std::vector<OpenRAVE::KinBodyConstPtr> DeserializeKinBodyList(std::string keyname, picojson::value val);
    std::vector<OpenRAVE::KinBody::LinkConstPtr> DeserializeLinkList(std::string keyname, picojson::value check_log_val);

};

}

#endif // CHECKER_RESULT_MODULE_H