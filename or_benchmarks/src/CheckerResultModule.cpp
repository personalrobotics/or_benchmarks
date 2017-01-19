#include <benchmarks/CheckerResultModule.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <string>
#include <exception>
#include <algorithm>


using namespace benchmarks;

/*
OpenRAVE Plugin that takes the log of a collision checker
in the format specified by or_stub_checker and replays
it with the collision checker of the environment.
The time and result is sent to the output stream
*/



CheckerResultModule::CheckerResultModule(OpenRAVE::EnvironmentBasePtr env)
    : ModuleBase(env) {

    RegisterCommand("EvaluateCheck", boost::bind(&CheckerResultModule::EvaluateCheck, this, _1, _2),
                    "Time the collision check");
}

CheckerResultModule::~CheckerResultModule() {

}

bool CheckerResultModule::EvaluateCheck(std::ostream &sout, std::istream &sin) {

    // Obtain checker for environment
    OpenRAVE::CollisionCheckerBasePtr checker_ptr = GetEnv()->GetCollisionChecker();

    if (!checker_ptr)
    {
        throw std::runtime_error("Environment does not have valid collision checker!");
    }

    // Result to output
    std::chrono::time_point<std::chrono::high_resolution_clock> start,end;
    double time_result = 0.0;
    bool check_result = false;


    // Now read the JSON string and parse it
    std::string check_log_json_string(std::istreambuf_iterator<char>(sin),{});


    picojson::value check_log_val;
    std::string err = picojson::parse(check_log_val,check_log_json_string);
    if(!err.empty()){
        throw OpenRAVE::openrave_exception(err);
    }

    // Conver to object
    if( ! check_log_val.is<picojson::object>()){
        throw OpenRAVE::openrave_exception("JSON stream is not an object!");
    }

    // First get methodname
    std::string methodname(check_log_val.get("methodname").get<std::string>());

    // Now do checks depending on methodname
    if (methodname == "CheckCollision_body_env")
    {
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        //Time check
        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(pbody);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_body1_body2")
    {
        OpenRAVE::KinBodyConstPtr pbody1 = DeserializeKinBody("body1",check_log_val);
        OpenRAVE::KinBodyConstPtr pbody2 = DeserializeKinBody("body2",check_log_val);

        //Time check
        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(pbody1,pbody2);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckSelfCollision")
    {
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        //Time check
        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckSelfCollision(pbody);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_link_env")
    {
        OpenRAVE::KinBody::LinkConstPtr plink = DeserializeLink("link",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(plink);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_link1_link2")
    {
        OpenRAVE::KinBody::LinkConstPtr plink1 = DeserializeLink("link1",check_log_val);
        OpenRAVE::KinBody::LinkConstPtr plink2 = DeserializeLink("link2",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(plink1, plink2);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_link_body")
    {
        OpenRAVE::KinBody::LinkConstPtr plink = DeserializeLink("link",check_log_val);
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(plink, pbody);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_link_with_exclusions")
    {
        OpenRAVE::KinBody::LinkConstPtr plink = DeserializeLink("link",check_log_val);

        std::vector<OpenRAVE::KinBodyConstPtr> vbodiesexcluded = DeserializeKinBodyList("bodies_excluded",check_log_val);
        std::vector<OpenRAVE::KinBody::LinkConstPtr> vlinksexcluded = DeserializeLinkList("links_excluded",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(plink, vbodiesexcluded, vlinksexcluded);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckCollision_body_with_exclusions")
    {
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        std::vector<OpenRAVE::KinBodyConstPtr> vbodiesexcluded = DeserializeKinBodyList("bodies_excluded",check_log_val);
        std::vector<OpenRAVE::KinBody::LinkConstPtr> vlinksexcluded = DeserializeLinkList("links_excluded",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckCollision(pbody, vbodiesexcluded, vlinksexcluded);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckStandaloneSelfCollision")
    {
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckStandaloneSelfCollision(pbody);
        end = std::chrono::high_resolution_clock::now();
    }

    else if (methodname == "CheckStandaloneSelfCollisionBaked")
    {
        OpenRAVE::KinBodyConstPtr pbody = DeserializeKinBody("body",check_log_val);

        start = std::chrono::high_resolution_clock::now();
        check_result = checker_ptr -> CheckStandaloneSelfCollision(pbody);
        end = std::chrono::high_resolution_clock::now();
    }

    else
    {
        throw OpenRAVE::openrave_exception("Unknown method - "+methodname);
    }

    std::chrono::duration<double> elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(end-start);
    time_result = elapsed_seconds.count();

    sout<<check_result<<" "<<time_result;


    return true;

}

OpenRAVE::KinBodyConstPtr CheckerResultModule::DeserializeKinBody(std::string keyname, picojson::value check_log_val)
{
    std::string body_name(check_log_val.get(keyname).get<std::string>());
    OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);

    if(!pbody){
        throw OpenRAVE::openrave_exception("Kinbody name invalid for environment");
    }

    return pbody;
}

OpenRAVE::KinBody::LinkConstPtr CheckerResultModule::DeserializeLink(std::string keyname, picojson::value check_log_val)
{
    std::string link_and_body(check_log_val.get(keyname).get<std::string>());

    size_t first_blank_pos = link_and_body.find(' ');

    std::string link_name = link_and_body.substr(0,first_blank_pos);
    std::string body_name = link_and_body.substr(first_blank_pos+1);

    OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);
    OpenRAVE::KinBody::LinkConstPtr plink = pbody -> GetLink(link_name);

    if(!plink){
        throw OpenRAVE::openrave_exception("Link name "+link_name+" not in kinbody "+body_name);
    }

    return plink;
}


std::vector<OpenRAVE::KinBodyConstPtr> CheckerResultModule::DeserializeKinBodyList(std::string keyname, picojson::value check_log_val)
{
    picojson::value::array val_arr(check_log_val.get(keyname).get<picojson::value::array>());
    std::vector<OpenRAVE::KinBodyConstPtr> vbodies;

    for (auto val : val_arr)
    {
        std::string body_name(val.get<std::string>());
        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);
        if(!pbody){
            throw OpenRAVE::openrave_exception("Kinbody name "+body_name+" invalid for environment");
        }
        vbodies.push_back(pbody);
    }

    return vbodies;
}


std::vector<OpenRAVE::KinBody::LinkConstPtr> CheckerResultModule::DeserializeLinkList(std::string keyname, picojson::value check_log_val)
{
    picojson::value::array val_arr(check_log_val.get(keyname).get<picojson::value::array>());
    std::vector<OpenRAVE::KinBody::LinkConstPtr> vlinks;

    for (auto val : val_arr)
    {
        std::string link_and_body(val.get<std::string>());

        size_t first_blank_pos = link_and_body.find(' ');

        std::string link_name = link_and_body.substr(0,first_blank_pos);
        std::string body_name = link_and_body.substr(first_blank_pos+1);

        OpenRAVE::KinBodyConstPtr pbody = GetEnv() -> GetKinBody(body_name);
        OpenRAVE::KinBody::LinkConstPtr plink = pbody -> GetLink(link_name);

        if(!plink){
            throw OpenRAVE::openrave_exception("Link name "+link_name+" not in kinbody "+body_name);
        }

        vlinks.push_back(plink);
    }

    return vlinks;
}