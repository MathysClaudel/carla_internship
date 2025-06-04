// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <carla/PythonUtil.h>
#include <carla/client/ClientSideSensor.h>
#include <carla/client/LaneInvasionSensor.h>
#include <carla/client/Sensor.h>
#include <carla/client/ServerSideSensor.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/stl_iterator.hpp>

namespace bp = boost::python;

static void SubscribeToStream(carla::client::Sensor &self, bp::object callback) {
  self.Listen(MakeCallback(std::move(callback)));
}

static void SubscribeToGBuffer(
  carla::client::ServerSideSensor &self,
  uint32_t GBufferId,
  bp::object callback) {
  self.ListenToGBuffer(GBufferId, MakeCallback(std::move(callback)));
}

// Wrapper optionnel (inutile si vous utilisez vector_indexing_suite + implicit_convertible)
static void py_SetIgnoredActors(carla::client::ServerSideSensor &sensor, bp::object ids_obj) {
  std::vector<unsigned int> ids;
  bp::stl_input_iterator<unsigned int> begin(ids_obj), end;
  ids.assign(begin, end);
  sensor.SetLidarIgnoredActors(ids);
}

void export_sensor() {
  using namespace boost::python;
  namespace cc = carla::client;

  // 1) Exposer std::vector<unsigned int> auprès de Boost.Python
  class_<std::vector<unsigned int>>("UIntVector")
    .def(vector_indexing_suite<std::vector<unsigned int>>());

  // 3) Exposer le reste de vos classes
  class_<cc::Sensor, bases<cc::Actor>, boost::noncopyable, boost::shared_ptr<cc::Sensor>>
      ("Sensor", no_init)
    .def("listen", &SubscribeToStream, (arg("callback")))
    .def("is_listening", &cc::Sensor::IsListening)
    .def("stop", &cc::Sensor::Stop)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::ServerSideSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::ServerSideSensor>>
      ("ServerSideSensor", no_init)
    .def("listen_to_gbuffer", &SubscribeToGBuffer, (arg("gbuffer_id"), arg("callback")))
    .def("is_listening_gbuffer", &cc::ServerSideSensor::IsListeningGBuffer, (arg("gbuffer_id")))
    .def("stop_gbuffer", &cc::ServerSideSensor::StopGBuffer, (arg("gbuffer_id")))
    .def("enable_for_ros", &cc::ServerSideSensor::EnableForROS)
    .def("disable_for_ros", &cc::ServerSideSensor::DisableForROS)
    .def("is_enabled_for_ros", &cc::ServerSideSensor::IsEnabledForROS)
    .def("send", &cc::ServerSideSensor::Send, (arg("message")))
    // Avec implicit_convertible, on peut maintenant passer une `list[int]` directement :
    .def("set_ignored_actors", &py_SetIgnoredActors, (bp::arg("ids")))
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::ClientSideSensor, bases<cc::Sensor>, boost::noncopyable, boost::shared_ptr<cc::ClientSideSensor>>
      ("ClientSideSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;

  class_<cc::LaneInvasionSensor, bases<cc::ClientSideSensor>, boost::noncopyable, boost::shared_ptr<cc::LaneInvasionSensor>>
      ("LaneInvasionSensor", no_init)
    .def(self_ns::str(self_ns::self))
  ;
}
