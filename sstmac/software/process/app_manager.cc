#include <sstmac/common/runtime.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/software/process/app.h>
#include <sstmac/software/process/app_manager.h>
#include <sstmac/hardware/interconnect/interconnect_fwd.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/keyword_registration.h>

ImplementFactory(sstmac::sw::app_manager);
RegisterNamespaces("app_manager");

namespace sstmac {
namespace sw {

std::map<int, app_manager*> app_manager::static_app_managers_;

app_manager::app_manager()
{
}

app_manager::~app_manager()
{
  delete app_template_;
}

hw::node*
app_manager::node_at(node_id nid) const
{
  return interconn_->node_at(nid);
}

void
app_manager::set_interconnect(hw::interconnect* interconn)
{
  interconn_ = interconn;
  top_ = interconn->topol();
}

void
app_manager::init_factory_params(sprockit::sim_parameters* params)
{
  appname_ = params->get_param("name");

  if (params->has_param("core_affinities")) {
    params->get_vector_param("core_affinities", core_affinities_);
  }

  app_template_ = sw::app_factory::get_value(appname_, params);

  STATIC_INIT_INTERCONNECT(params)
}

app_manager*
app_manager::static_app_manager(int aid, sprockit::sim_parameters* params)
{
  if (!static_app_managers_[aid]){
    std::string app_namespace = sprockit::printf("app%d", aid);
    sprockit::sim_parameters* app_params = params->top_parent()->get_namespace(app_namespace);
    app_manager* mgr = app_manager_factory::get_optional_param(
          "launch_type", "skeleton", app_params, app_id(aid), 0/*no parallel runtime*/);
    static_app_managers_[aid] = mgr;
    runtime::register_app_manager(app_id(aid), mgr);
    mgr->allocate_and_index_jobs();
  }
  return static_app_managers_[aid];
}

node_id
app_manager::node_for_task(sw::task_id tid) const
{
  task_to_node_map::const_iterator it = nodeids_.find(tid);
  if (it == nodeids_.end()) {
    spkt_throw_printf(sprockit::value_error,
                     "sstmac_runtime::get_node: can not find task %d",
                     int(tid));
  }
  return it->second;
}

}
}

