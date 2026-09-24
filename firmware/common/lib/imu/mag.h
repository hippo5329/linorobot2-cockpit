// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAG_CONFIG_H
#define MAG_CONFIG_H

// include the header of your new driver here similar to default_mag.h
#include "default_mag.h"

// now you can create a config constant that you can use in lino_base_config.h






#ifndef MAG // use simulated mag when there is no real mag
    #define USE_SIM_MAG
    #define MAG SimMAG
#endif

// No `#define MAG <class>` chain below any more.
//
// It mapped USE_MPU6050_MAG, USE_QMI8658_MAG and the rest onto a single typedef
// so that one driver could be named at COMPILE time -- the thing sensor_factory
// exists to replace. Every driver class above is compiled into every image and
// createIMU()/createMAG() dispatch on a name from the env, which the I2C probe
// can overrule. Nothing has referenced the typedefs since test_motors.cpp moved
// to the factory, so the selection chain was dead code that still made a board
// look like a build-time choice.

#endif
