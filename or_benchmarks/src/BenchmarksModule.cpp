/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <openrave/plugin.h>
#include <benchmarks/CollisionCheckingBenchmark.h>
#include <benchmarks/KinematicBenchmarks.h>
#include <benchmarks/CheckerResultModule.h>

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, std::string const &name,
                                          std::istream& sinput, EnvironmentBasePtr env)
{
    RAVELOG_INFO("[CollisionCheckingModule] Creating interface.\n");
	if(type == PT_Module && name == "collisioncheckingbenchmark"){
		RAVELOG_INFO("[CollisionCheckingModule] Generating collision checking benchmark plugin.\n");
        return OpenRAVE::InterfaceBasePtr(new benchmarks::CollisionCheckingBenchmark(env));
	}else if(type == PT_Module && name == "kinematicbenchmarks"){
		RAVELOG_INFO("[CollisionCheckingModule] Generating kinematic benchmarks plugin.\n");
        return OpenRAVE::InterfaceBasePtr(new benchmarks::KinematicBenchmarks(env));
	}else if(type == PT_Module && name == "checkerresultmodule"){
    RAVELOG_INFO("[CheckerResultModule] Generating checker evaluator plugin.\n");
        return OpenRAVE::InterfaceBasePtr(new benchmarks::CheckerResultModule(env));
  }

    RAVELOG_INFO("[CollisionCheckingModule] Generating empty pointer.\n");

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO &pinfo)
{
	pinfo.interfacenames[PT_Module].push_back("CollisionCheckingBenchmark");
	pinfo.interfacenames[PT_Module].push_back("KinematicBenchmarks");
  pinfo.interfacenames[PT_Module].push_back("CheckerResultModule");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("[CollisionCheckingModule] Destroying plugin.\n");
}
