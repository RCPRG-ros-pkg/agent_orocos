/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/os/main.h>

#include <rtt_rosclock/rtt_rosclock.h>

#include <math.h>
#include <algorithm>

#include <vector>
#include <set>
#include <string>
#include <pthread.h>

#include "common_behavior/input_data.h"
#include "common_behavior/master_service_requester.h"
#include "common_behavior/master_service.h"
#include "common_behavior/abstract_state.h"

using namespace RTT;

typedef std::shared_ptr<common_behavior::BehaviorBase > BehaviorBasePtr;
typedef std::shared_ptr<const common_behavior::BehaviorBase > BehaviorBaseConstPtr;

typedef std::shared_ptr<common_behavior::StateBase > StateBasePtr;
typedef std::shared_ptr<const common_behavior::StateBase > StateBaseConstPtr;

class DiagBehaviorSwitch {
public:
    enum Reason {INVALID, INIT, STOP, ERROR};
    int id_;
    RTT::os::TimeService::nsecs time_;
    Reason reason_;
    common_behavior::PredicateListPtr pred_;

    static const std::string& getReasonStr(Reason r) {
        static const std::string inv("INV");
        static const std::string init("INIT");
        static const std::string stop("STOP");
        static const std::string err("ERR");
        switch (r) {
        case INIT:
            return init;
        case STOP:
            return stop;
        case ERROR:
            return err;
        }
        return inv;
    }

    const std::string& getReasonStr() const {
        return getReasonStr(reason_);
    }
};

class DiagBehaviorSwitchHistory {
public:
    DiagBehaviorSwitchHistory()
        : idx_(0)
    {}

    void addBehaviorSwitch(int new_behavior_id, RTT::os::TimeService::nsecs time, DiagBehaviorSwitch::Reason reason, common_behavior::PredicateListConstPtr pred = common_behavior::PredicateListConstPtr()) {
        if (h_.size() == 0) {
            return;
        }
        h_[idx_].id_ = new_behavior_id;
        h_[idx_].time_ = time;
        h_[idx_].reason_ = reason;
        if (pred) {
            *(h_[idx_].pred_) = *pred;
        }
        idx_ = (idx_+1) % h_.size();
    }

    void setSize(size_t size, RTT::OperationCaller<common_behavior::PredicateListPtr()> common_behavior::MasterServiceRequester::*func, common_behavior::MasterServiceRequester& a) {
        h_.resize(size);
        for (int i = 0; i < h_.size(); ++i) {
            h_[i].id_ = -1;
            h_[i].reason_ = DiagBehaviorSwitch::INVALID;
            h_[i].pred_ = (a.*func)();
        }
        idx_ = 0;
    }

    bool getBehaviorSwitchHistory(int idx, DiagBehaviorSwitch &ss) const {
        if (idx >= h_.size() || h_.size() == 0) {
            return false;
        }
        int i = (idx_-idx-1+h_.size()*2) % h_.size();
        if (h_[i].reason_ == DiagBehaviorSwitch::INVALID) {
            return false;
        }
        ss = h_[i];
        return true;
    }

private:

    std::vector<DiagBehaviorSwitch> h_;
    int idx_;
};
/*
bool componentsInConflict(const std::string& c1, const std::string& c2, const std::set<std::pair<std::string, std::string > >& conflicting_components) {
    std::pair<std::string, std::string > p1(c1, c2);
    std::pair<std::string, std::string > p2(c2, c1);
    if (conflicting_components.find(p1) == conflicting_components.end() &&
        conflicting_components.find(p2) == conflicting_components.end()) {
        return false;
    }
    return true;
}

bool behaviorsInConflict(const BehaviorBaseConstPtr &b1, const BehaviorBaseConstPtr &b2, const std::set<std::pair<std::string, std::string > >& conflicting_components) {
    const std::vector<std::string >& b1_comp_vec = b1->getRunningComponents();
    const std::vector<std::string >& b2_comp_vec = b2->getRunningComponents();
    for (int i = 0; i < b1_comp_vec.size(); ++i) {
        for (int j = 0; j < b2_comp_vec.size(); ++j) {
            if (componentsInConflict(b1_comp_vec[i], b2_comp_vec[j], conflicting_components)) {
                return true;
            }
        }
    }
    return false;
}
*/
/*
void recursiveAddBehavior(const std::vector<BehaviorBasePtr >& behaviors, int start_idx,
                            std::vector<std::vector<int> >& out, std::vector<int > bv = std::vector<int >()) {
//    if (os->isComplete()) {
//        out.push_back(bv);
//        return;
//    }

//TODO:
    for (int i = start_idx; i < behaviors.size(); ++i) {
        bool compatible = true;
        for (int j = 0; j < bv.size(); ++j) {
            if (behaviorsInConflict(behaviors[i], behaviors[bv[j]], conflicting_components)) {
                compatible = false;
                break;
            }
        }
        if ( compatible ) {
//            os->add(behaviors[i]->getOutputScope());
            bv.push_back(i);
            recursiveAddBehavior(behaviors, i+1, os, out, bv);
            bv.pop_back();
//            os->substract(behaviors[i]->getOutputScope());
        }
    }
}
*/
class MasterComponent: public RTT::TaskContext {
public:
    explicit MasterComponent(const std::string &name);

    bool configureHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

    bool addConmanScheme(RTT::TaskContext* scheme);

    void setThreadName(const std::string& thread_name);

private:
    BehaviorBasePtr getBehavior(const std::string& name) const;
    StateBasePtr getState(const std::string& name) const;
    bool addBehavior(const BehaviorBasePtr& ptr);
    bool removeBehavior(const BehaviorBasePtr& ptr);
    void calculateConflictingComponents();
    void printCurrentBehaviors() const;
    bool isCurrentBehavior(const std::string& behavior_name) const;
    bool isCurrentBehavior(int behavior_idx) const;
    int currentBehaviorsCount() const;
    bool isGraphOk() const;

    std::vector<StateBasePtr > states_;
    StateBasePtr current_state_;
    std::map<std::string, std::vector<BehaviorBasePtr > > state_behaviors_;
    std::map<std::string, int > state_graphs_;

    std::vector<BehaviorBasePtr > behaviors_;

    // pointer to conman scheme TaskContext
    TaskContext *scheme_;

    // conman scheme operations
    RTT::OperationCaller<bool(const std::string &)> hasBlock_;
    RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)> addGraphConfiguration_;
    RTT::OperationCaller<bool(int)> switchToConfiguration_;

    std::vector<TaskContext* > scheme_peers_;
    std::vector<const TaskContext* > scheme_peers_const_;
    std::set<std::string > switchable_components_;
    std::vector<std::vector<bool > > is_running_in_behavior_;

    bool first_step_;

    RTT::base::DataObjectLockFree<DiagBehaviorSwitchHistory > diag_bs_sync_;
    DiagBehaviorSwitchHistory diag_ss_rt_;

    boost::shared_ptr<common_behavior::MasterServiceRequester > master_service_;

    boost::shared_ptr<common_behavior::InputData > in_data_;

    common_behavior::PredicateListPtr predicate_list_;

    RTT::Seconds last_exec_time_, last_exec_period_;
    RTT::os::TimeService::nsecs last_update_time_;
    RTT::os::TimeService::Seconds scheme_time_;

    int behavior_switch_history_length_;

    std::set<std::pair<std::string, std::string > > conflicting_components_;

    int counter_;

    double interval1_;
    double interval2_;
    double interval3_;
    double interval4_;
    double interval5_;

    std::string thread_name_;
};

void MasterComponent::setThreadName(const std::string& thread_name) {
    thread_name_ = thread_name;
    RTT::log(RTT::Info) << "master component thread name: " << thread_name_ << RTT::endlog();
}

MasterComponent::MasterComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , behavior_switch_history_length_(5)
    , scheme_time_(0)
{
    this->addOperation("getDiag", &MasterComponent::getDiag, this, RTT::ClientThread);

    this->addOperation("addConmanScheme", &MasterComponent::addConmanScheme, this, RTT::ClientThread);

    this->addOperation("setThreadName", &MasterComponent::setThreadName, this, RTT::ClientThread);

    addProperty("behavior_switch_history_length", behavior_switch_history_length_);
}

std::string MasterComponent::getDiag() {
    std::ostringstream strs;

    strs << "<mcd>";
    strs << "<h>";

    DiagBehaviorSwitchHistory ss;
    diag_bs_sync_.Get(ss);


    for (int i = 0; ; ++i) {
        DiagBehaviorSwitch s;
        if (!ss.getBehaviorSwitchHistory(i, s)) {
            break;
        }
        RTT::os::TimeService::Seconds switch_interval = RTT::nsecs_to_Seconds(last_update_time_ - s.time_);

        std::string err_str;
        if (s.pred_) {
            err_str = master_service_->getPredicatesStr(s.pred_);
        }

        std::string behavior_name;
        if (s.id_ >= 0) {
            behavior_name = states_[s.id_]->getShortStateName();
        }
        else {
            behavior_name = "INV_BEH";
        }
        strs << "<ss n=\"" << behavior_name << "\" r=\""
             << s.getReasonStr() << "\" t=\"" << switch_interval << "\" e=\""
             << err_str << "\" />";
    }

    strs << "</h>";

    strs << "<pr v=\"" << master_service_->getPredicatesStr(predicate_list_) << "\" />";

    strs << "<p>" << last_exec_period_ << "</p>";
    strs << "<t_tf>" << scheme_time_ << "</t_tf>";

    strs << "<int1>" << interval1_ << "</int1>";
    strs << "<int2>" << interval2_ << "</int2>";
    strs << "<int3>" << interval3_ << "</int3>";
    strs << "<int4>" << interval4_ << "</int4>";
    strs << "<int5>" << interval5_ << "</int5>";

    strs << "</mcd>";

    return strs.str();
}

bool MasterComponent::addConmanScheme(RTT::TaskContext* scheme) {
    scheme_ = scheme;
    return scheme_->setActivity( new RTT::extras::SlaveActivity(this->getActivity(), scheme_->engine()));
}

void MasterComponent::calculateConflictingComponents() {
    for (int i = 0; i < scheme_peers_.size(); ++i) {
        Service::shared_ptr sv = scheme_peers_[i]->provides();
        RTT::Service::PortNames comp_ports = sv->getPortNames();
        for (int j = 0; j < comp_ports.size(); ++j) {
            RTT::base::InputPortInterface* ipi = dynamic_cast<RTT::base::InputPortInterface* >( sv->getPort(comp_ports[j]) );
            // check input ports only
            if (!ipi) {
                continue;
            }
            std::vector<std::string > comp_out;
            std::vector<std::string > port_out;
            std::list<internal::ConnectionManager::ChannelDescriptor> chns = ipi->getManager()->getConnections();
            for(std::list<internal::ConnectionManager::ChannelDescriptor>::iterator k = chns.begin(); k != chns.end(); k++) {
                base::ChannelElementBase::shared_ptr bs = k->get<1>();

                if(bs->getInputEndPoint()->getPort() != 0) {
                    if (bs->getInputEndPoint()->getPort()->getInterface() != 0 ){
                        comp_out.push_back( bs->getInputEndPoint()->getPort()->getInterface()->getOwner()->getName() );
                        port_out.push_back( bs->getInputEndPoint()->getPort()->getName() );
                    }
                }
            }

            if (comp_out.size() > 1) {
                RTT::log(RTT::Info) << "Conflicting components for input port " << scheme_peers_[i]->getName() << "." << comp_ports[j] << ":" << RTT::endlog();
                for (int k = 0; k < comp_out.size(); ++k) {
                    RTT::log(RTT::Info) << "    \'" << comp_out[k] << "\' (port: \'" << port_out[k] << "\')" << RTT::endlog();
                }
            }

            for (int k = 0; k < comp_out.size(); ++k) {
                for (int l = k+1; l < comp_out.size(); ++l) {
                    std::pair<std::string, std::string > p1(comp_out[k], comp_out[l]);
                    std::pair<std::string, std::string > p2(comp_out[l], comp_out[k]);
                    if (conflicting_components_.find(p1) == conflicting_components_.end() &&
                        conflicting_components_.find(p2) == conflicting_components_.end()) {
                        conflicting_components_.insert(p1);
                    }
                }
            }
        }
    }
    RTT::log(RTT::Info) << "Conflicting components pairs:" << RTT::endlog();
    for (std::set<std::pair<std::string, std::string > >::const_iterator it = conflicting_components_.begin(); it != conflicting_components_.end(); ++it) {
        RTT::log(RTT::Info) << "    " << it->first << ", " << it->second << RTT::endlog();
    }
}

BehaviorBasePtr MasterComponent::getBehavior(const std::string& name) const {
    for (int i = 0; i < behaviors_.size(); ++i) {
        if (behaviors_[i]->getName() == name) {
            return behaviors_[i];
        }
    }
    return BehaviorBasePtr();
}

StateBasePtr MasterComponent::getState(const std::string& name) const {
    for (int i = 0; i < states_.size(); ++i) {
        if (states_[i]->getStateName() == name) {
            return states_[i];
        }
    }
    return StateBasePtr();
}

bool MasterComponent::configureHook() {
    Logger::In in("MasterComponent::configureHook");

    master_service_ = this->getProvider<common_behavior::MasterServiceRequester >("master");
    if (!master_service_) {
        RTT::log(RTT::Error) << "Unable to load common_behavior::MasterService" << RTT::endlog();
        return false;
    }

    predicate_list_ = master_service_->allocatePredicateList();
    if (!predicate_list_) {
        Logger::log() << Logger::Error << "could not allocate predicate list" << Logger::endl;
        return false;
    }

    //
    // retrieve states list
    //
    std::vector<std::string > state_names_ = master_service_->getStates();

    Logger::log() << Logger::Info << "Used states:" << Logger::endl;
    for (int i = 0; i < state_names_.size(); ++i) {
        auto s_ptr = common_behavior::StateFactory::Instance()->Create( state_names_[i] );
        if (s_ptr) {
            Logger::log() << Logger::Info << "    " << state_names_[i] << ", short name: " << s_ptr->getShortStateName() << Logger::endl;
            states_.push_back(s_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown state: " << state_names_[i] << Logger::endl;
            return false;
        }
    }

    //
    // retrieve behaviors list
    //
    std::vector<std::string > behavior_names_ = master_service_->getBehaviors();

    Logger::log() << Logger::Info << "Known behaviors: " << Logger::endl;
    for (auto it = common_behavior::BehaviorFactory::Instance()->getBehaviors().begin();
        it != common_behavior::BehaviorFactory::Instance()->getBehaviors().end(); ++it)
    {
        Logger::log() << Logger::Info << "    " << it->first << Logger::endl;
    }

    Logger::log() << Logger::Info << "Used behaviors:" << Logger::endl;
    for (int i = 0; i < behavior_names_.size(); ++i) {
        auto b_ptr = common_behavior::BehaviorFactory::Instance()->Create( behavior_names_[i] );
        if (b_ptr) {
            Logger::log() << Logger::Info << "    " << behavior_names_[i] << ", short name: " << b_ptr->getShortName() << Logger::endl;
            behaviors_.push_back(b_ptr);
        }
        else {
            Logger::log() << Logger::Error << "unknown behavior: " << behavior_names_[i] << Logger::endl;
            return false;
        }
    }

    Logger::log() << Logger::Info << "initial state: " << master_service_->getInitialState() << Logger::endl;

    // create map of behaviors used in each state
    for (int i = 0; i < states_.size(); ++i) {
        if (states_[i]->getStateName() == master_service_->getInitialState()) {
            current_state_ = states_[i];
        }
        const std::vector<std::string>& behavior_names = states_[i]->getBehaviorNames();
        std::vector<BehaviorBasePtr > state_behaviors;
        for (int j = 0; j < behavior_names.size(); ++j) {
            state_behaviors.push_back( getBehavior(behavior_names[j]) );
        }

        state_behaviors_[states_[i]->getStateName()] = state_behaviors;
    }

    if (!current_state_) {
        Logger::log() << Logger::Error << "could not select initial state: " << master_service_->getInitialState() << Logger::endl;
        return false;
    }

    // retrieve the vector of peers of conman scheme
    TaskContext::PeerList scheme_peers_names = scheme_->getPeerList();
    for (int pi = 0; pi < scheme_peers_names.size(); ++pi) {
        scheme_peers_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
        scheme_peers_const_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
    }
/*
    // prepare list of conflicting components
    calculateConflictingComponents();

    // calculate list of all possible complete behaviors
    common_behavior::OutputScopeBasePtr os = master_service_->allocateOutputScope();
//TODO: uncomment
//    recursiveAddBehavior(behaviors_, 0, os, possible_behaviors_);
    Logger::log() << Logger::Info << "possible complete behaviors: " << Logger::endl;
    for (int i = 0; i < possible_behaviors_.size(); ++i) {
        std::string str;
        std::string sep = "";
        for (int j = 0; j < possible_behaviors_[i].size(); ++j) {
            str += sep + behaviors_[possible_behaviors_[i][j]]->getName();
            sep = ", ";
        }
        Logger::log() << Logger::Info << "    " << str << Logger::endl;
    }
*/
    diag_ss_rt_.setSize(behavior_switch_history_length_, &common_behavior::MasterServiceRequester::allocatePredicateList, *master_service_);

    diag_bs_sync_.data_sample(diag_ss_rt_);
    diag_bs_sync_.Set(diag_ss_rt_);

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());


    // get names of all components that are needed for all behaviors
    for (int i = 0; i < behaviors_.size(); ++i) {
        const std::vector<std::string >& comp_vec = behaviors_[i]->getRunningComponents();
        for (int j = 0; j < comp_vec.size(); ++j) {
            if (hasBlock_( comp_vec[j] )) {
                switchable_components_.insert( comp_vec[j] );
            }
            else {
                Logger::log() << Logger::Error << "could not find a component \'" << comp_vec[j] << "\' in the scheme blocks list" << Logger::endl;
                return false;
            }
        }
    }

    std::vector<std::string > all_converter_components;
    std::vector<std::string > always_running_converter_components;

    std::map<std::string, std::pair<std::string, std::string > > map_converters_components;

    // get all converter components
    for (int i = 0; i < scheme_peers_.size(); ++i) {
        RTT::OperationInterfacePart *isDataConverterOp = scheme_peers_[i]->getOperation("isDataConverter");
        if (!isDataConverterOp) {
            continue;
        }
        RTT::OperationCaller<bool()> isDataConverter = RTT::OperationCaller<bool()>(isDataConverterOp);
        if (isDataConverter()) {
            const std::string& converter_name = scheme_peers_[i]->getName();
            all_converter_components.push_back(converter_name);
            Logger::log() << Logger::Info << "Found data converter component: " << converter_name << Logger::endl;

            std::string comp_in;
            std::string comp_out;

            std::list<internal::ConnectionManager::ChannelDescriptor> chns = scheme_peers_[i]->getPort("data_INPORT")->getManager()->getConnections();
            if (chns.empty()) {
                Logger::log() << Logger::Error << "converter component is not connected (could not get channels): " << converter_name << Logger::endl;
                return false;
            }
            for (std::list<internal::ConnectionManager::ChannelDescriptor>::iterator k = chns.begin(); k != chns.end(); k++){
                base::ChannelElementBase::shared_ptr bs = k->get<1>();
                if (bs->getInputEndPoint()->getPort() != 0){
                    if (bs->getInputEndPoint()->getPort()->getInterface() != 0 ){
                        comp_in = bs->getInputEndPoint()->getPort()->getInterface()->getOwner()->getName();
                        break;
/*                        std::map<std::string, std::vector<std::string > >::iterator m_it = map_components_converters.find(comp_in);
                        if (m_it != map_components_converters.end()) {
                            m_it->second.push_back( converter_name );

                        }
                        else {
                            std::vector<std::string > vec;
                            vec.push_back( converter_name );
                            map_components_converters.insert( std::make_pair(comp_in, vec) );
                        }
                        if (switchable_components_.find(comp_in) == switchable_components_.end()) {
                            always_running_converter_components.push_back(converter_name);
                        }
*/
                    }
                    else{
                        Logger::log() << Logger::Error << "converter component is not connected (could not get interface): " << converter_name << Logger::endl;
                        return false;
                    }
                }
                else {
                    Logger::log() << Logger::Error << "converter component is not connected (could not get port): " << converter_name << Logger::endl;
                    return false;
                }
            }

            chns = scheme_peers_[i]->getPort("data_OUTPORT")->getManager()->getConnections();
            if (chns.empty()) {
                Logger::log() << Logger::Error << "converter component is not connected (could not get channels): " << converter_name << Logger::endl;
                return false;
            }
            for (std::list<internal::ConnectionManager::ChannelDescriptor>::iterator k = chns.begin(); k != chns.end(); k++){
                base::ChannelElementBase::shared_ptr bs = k->get<1>();
                if (bs->getOutputEndPoint()->getPort() != 0){
                    if (bs->getOutputEndPoint()->getPort()->getInterface() != 0 ){
                        comp_out = bs->getOutputEndPoint()->getPort()->getInterface()->getOwner()->getName();
                        break;
/*                        std::map<std::string, std::vector<std::string > >::iterator m_it = map_components_converters.find(comp_out);
                        if (m_it != map_components_converters.end()) {
                            m_it->second.push_back( converter_name );

                        }
                        else {
                            std::vector<std::string > vec;
                            vec.push_back( converter_name );
                            map_components_converters.insert( std::make_pair(comp_out, vec) );
                        }
                        if (switchable_components_.find(comp_out) == switchable_components_.end()) {
                            always_running_converter_components.push_back(converter_name);
                        }
*/
                    }
                    else{
                        Logger::log() << Logger::Error << "converter component is not connected (could not get interface): " << converter_name << Logger::endl;
                        return false;
                    }
                }
                else {
                    Logger::log() << Logger::Error << "converter component is not connected (could not get port): " << converter_name << Logger::endl;
                    return false;
                }
            }

            Logger::log() << Logger::Error << "converter '" << converter_name << "' connects components '" << comp_in << "' and '" << comp_out << "'" << Logger::endl;
            map_converters_components.insert( std::make_pair(converter_name, std::make_pair(comp_in, comp_out)) );

            if (switchable_components_.find(comp_in) == switchable_components_.end() && switchable_components_.find(comp_out) == switchable_components_.end()) {
                always_running_converter_components.push_back(converter_name);
            }
        }
    }

    for (int i = 0; i < all_converter_components.size(); ++i) {
        if (hasBlock_( all_converter_components[i] )) {
            switchable_components_.insert( all_converter_components[i] );
        }
        else {
            Logger::log() << Logger::Error << "could not find a component \'" << all_converter_components[i] << "\' in the scheme blocks list" << Logger::endl;
            return false;
        }
    }

/*
    std::string switchable_converter_components_str;
    for (int i = 0; i < switchable_converter_components.size(); ++i) {
        if (hasBlock_( switchable_converter_components[i] )) {
            switchable_components_.insert( switchable_converter_components[i] );
            switchable_converter_components_str = switchable_converter_components_str + "'" + switchable_converter_components[i] + "', ";
        }
    }

    Logger::log() << Logger::Error << "switchable converter components: " << switchable_converter_components_str << Logger::endl;

    std::string always_running_converter_components_str;
    for (int i = 0; i < always_running_converter_components.size(); ++i) {
        switchable_components_.insert( always_running_converter_components[i] );
        always_running_converter_components_str = always_running_converter_components_str + "'" + always_running_converter_components[i] + "', ";
    }
    Logger::log() << Logger::Error << "always running converter components: " << always_running_converter_components_str << Logger::endl;
//*/

    std::string switchable_components_str;
    for (std::set<std::string >::const_iterator it = switchable_components_.begin(); it != switchable_components_.end(); ++it) {
        switchable_components_str = switchable_components_str + (switchable_components_str.empty()?"":", ") + (*it);
    }
    Logger::log() << Logger::Info << "switchable components: " << switchable_components_str << Logger::endl;



    for (std::set<std::string >::const_iterator it = switchable_components_.begin(); it != switchable_components_.end(); ++it) {
        if (!hasBlock_( *it )) {
            Logger::log() << Logger::Error << "could not find a component \'" << (*it) << "\' in the scheme blocks list" << Logger::endl;
            return false;
        }
    }

    RTT::OperationInterfacePart *addGraphConfigurationOp = scheme_->getOperation("addGraphConfiguration");
    if (addGraphConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation addGraphConfiguration" << Logger::endl;
        return false;
    }

    addGraphConfiguration_ = RTT::OperationCaller<bool(int, const std::vector<std::string>&, const std::vector<std::string>&)>(
        addGraphConfigurationOp, scheme_->engine());

    RTT::OperationInterfacePart *switchToConfigurationOp = scheme_->getOperation("switchToConfiguration");
    if (switchToConfigurationOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation switchToConfiguration" << Logger::endl;
        return false;
    }

    switchToConfiguration_ = RTT::OperationCaller<bool(int)>(
        switchToConfigurationOp, scheme_->engine());
/*
    for (std::map<std::string, std::vector<std::string > >::const_iterator it = map_components_converters.begin(); it != map_components_converters.end(); ++it) {
        std::string converters;
        for (int i = 0; i < it->second.size(); ++i) {
            converters = converters + "'" + it->second[i] + "', ";
        }
        Logger::log() << Logger::Info << "component '" << it->first << "' outputs are connected to converters: " << converters << Logger::endl;
        
    }
//*/
    Logger::log() << Logger::Info << "conman graph configurations:" << Logger::endl;
    for (int i = 0; i < states_.size(); ++i) {
        const std::vector<BehaviorBasePtr >& state_behaviors = state_behaviors_[states_[i]->getStateName()];
        std::vector<std::string > vec_running;
        for (int j = 0; j < state_behaviors.size(); ++j) {
            const std::vector<std::string >& const_v = state_behaviors[j]->getRunningComponents();
            std::vector<std::string > v = state_behaviors[j]->getRunningComponents();

            for (std::map<std::string, std::pair<std::string, std::string > >::iterator it = map_converters_components.begin(); it != map_converters_components.end(); ++it) {
                const std::string& comp_in = it->second.first;
                const std::string& comp_out = it->second.second;
                bool running_in = (std::find(const_v.begin(), const_v.end(), comp_in) != const_v.end());
                bool running_out = (std::find(const_v.begin(), const_v.end(), comp_out) != const_v.end());
                bool always_in = (std::find(always_running_converter_components.begin(), always_running_converter_components.end(), comp_in) != always_running_converter_components.end());
                bool always_out = (std::find(always_running_converter_components.begin(), always_running_converter_components.end(), comp_out) != always_running_converter_components.end());
//                if ((running_in && running_out) || (running_in && always_out) || (always_in && running_out)) {
                if (running_in || always_in || running_out || always_out) {
                    v.push_back(it->first);
                }
            }

//            for (int k = 0; k < const_v.size(); ++k) {
//                std::map<std::string, std::vector<std::string > >::iterator m_it = map_components_converters.find(const_v[k]);
//                if (m_it != map_components_converters.end()) {
//                    v.insert(v.end(), m_it->second.begin(), m_it->second.end());
//                }
//            }

            v.insert(v.end(), always_running_converter_components.begin(), always_running_converter_components.end());
            v.insert(v.end(), all_converter_components.begin(), all_converter_components.end());

            for (int k = 0; k < v.size(); ++k) {
                if (std::find(vec_running.begin(), vec_running.end(), v[k]) == vec_running.end()) {
                    if (hasBlock_( v[k] )) {
                        vec_running.push_back(v[k]);
                    }
                    else {
                        Logger::log() << Logger::Error << "conman scheme has no block: " << v[k] << Logger::endl;
                        return false;
                    }
                }
            }
        }

        std::set<std::string > comp_beh_set = std::set<std::string >(vec_running.begin(), vec_running.end());
        std::vector<bool > comp_beh_vec;
        for (int j = 0; j < scheme_peers_const_.size(); ++j) {
            const std::string& name = scheme_peers_const_[j]->getName();
            if (comp_beh_set.find(name) != comp_beh_set.end() || switchable_components_.find(name) == switchable_components_.end()) {
                comp_beh_vec.push_back(true);
            }
            else {
                comp_beh_vec.push_back(false);
            }
        }
        is_running_in_behavior_.push_back(comp_beh_vec);

        std::vector<std::string > vec_stopped;
        for (std::set<std::string >::const_iterator ic = switchable_components_.begin(); ic != switchable_components_.end(); ++ic) {
            bool is_running = false;
            for (int ir = 0; ir < vec_running.size(); ++ir) {
                if ( (*ic) == vec_running[ir] ) {
                    is_running = true;
                    break;
                }
            }
            if (!is_running) {
                vec_stopped.push_back( *ic );
            }
        }

        std::string str_running, str_stopped;
        for (int j = 0; j < vec_stopped.size(); ++j) {
            str_stopped += vec_stopped[j] + ", ";
        }
        for (int j = 0; j < vec_running.size(); ++j) {
            str_running += vec_running[j] + ", ";
        }

        Logger::log() << Logger::Info << i << " '" << states_[i]->getStateName() << "':  s:[" << str_stopped << "], r:[" << str_running << "]" << Logger::endl;

        addGraphConfiguration_(i, vec_stopped, vec_running);
        state_graphs_[states_[i]->getStateName()] = i;
    }

    in_data_ = master_service_->getDataSample();
    if (!in_data_) {
        RTT::log(RTT::Error) << "Unable to get InputData sample" << RTT::endlog();
        return false;
    }

    // run this function for proper initialization of predicate functions
    master_service_->calculatePredicates(in_data_, scheme_peers_const_, predicate_list_);

    return true;
}

bool MasterComponent::startHook() {
    first_step_ = true;
    return true;
}

void MasterComponent::stopHook() {
}

bool MasterComponent::isGraphOk() const {

    int current_graph_id = state_graphs_.find(current_state_->getStateName())->second;
    const std::vector<bool >& beh_running_vec = is_running_in_behavior_[current_graph_id];

    for (int i = 0; i < scheme_peers_const_.size(); ++i) {
        const std::string& name = scheme_peers_const_[i]->getName();
        RTT::TaskContext::TaskState state = scheme_peers_const_[i]->getTaskState();

        if (beh_running_vec[i] && state != RTT::TaskContext::Running) {
            Logger::log() << Logger::Error << "switchable component \'" << name << "\' should be running" << Logger::endl;
            return false;
        }

        if (!beh_running_vec[i] && state != RTT::TaskContext::Stopped) {
            Logger::log() << Logger::Error << "switchable component \'" << name << "\' should be stopped" << Logger::endl;
            return false;
        }
    }
    return true;
}

static void timespec_diff(struct timespec *start, struct timespec *stop,
                   struct timespec *result)
{
    if ((stop->tv_nsec - start->tv_nsec) < 0) {
        result->tv_sec = stop->tv_sec - start->tv_sec - 1;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
    } else {
        result->tv_sec = stop->tv_sec - start->tv_sec;
        result->tv_nsec = stop->tv_nsec - start->tv_nsec;
    }

    return;
}

void MasterComponent::updateHook() {
    // What time is it
    RTT::os::TimeService::nsecs now = RTT::os::TimeService::Instance()->getNSecs();
    RTT::os::TimeService::Seconds
        time = RTT::nsecs_to_Seconds(now),
        period = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs(last_update_time_));

    ros::Time time1 = rtt_rosclock::rtt_wall_now();

    master_service_->readBuffers();

    // Store update time
    last_update_time_ = now;

    // Compute statistics describing how often update is being called
    last_exec_period_ = time - last_exec_time_;
    last_exec_time_ = time;


    master_service_->initBuffersData(in_data_);

    master_service_->getBuffers(in_data_);
    counter_++;

    master_service_->writePorts(in_data_);

    bool behavior_switch = false;

    bool graphOk = true;
    if (first_step_) {
        first_step_ = false;
        behavior_switch = true;

        diag_ss_rt_.addBehaviorSwitch(state_graphs_[current_state_->getStateName()], now, DiagBehaviorSwitch::INIT);
        diag_bs_sync_.Set(diag_ss_rt_);
    }
    else {
        // conman graph is initialized in first iteration
        graphOk = isGraphOk();
    }

    master_service_->calculatePredicates(in_data_, scheme_peers_const_, predicate_list_);
    predicate_list_->CURRENT_BEHAVIOR_OK = graphOk;

    const std::vector<BehaviorBasePtr >& current_behaviors = state_behaviors_[current_state_->getStateName()];
    bool err_cond = false;
    for (int i = 0; i < current_behaviors.size(); ++i) {
        if (current_behaviors[i]->checkErrorCondition(predicate_list_)) {
            err_cond = true;
            break;
        }
    }

    bool stop_cond = false;

    if (!err_cond) {
        for (int i = 0; i < current_behaviors.size(); ++i) {
            if (current_behaviors[i]->checkStopCondition(predicate_list_)) {
                stop_cond = true;
                break;
            }
        }
    }

    predicate_list_->IN_ERROR = err_cond;
    ros::Time time2 = rtt_rosclock::rtt_wall_now();

    if (stop_cond || err_cond) {
        const std::string& next_state_name = current_state_->getNextState(predicate_list_);
//        Logger::log() << Logger::Info << "switching to new state: " << next_state_name
//            << Logger::endl;
        current_state_ = getState(next_state_name);
        if (!current_state_) {
            Logger::log() << Logger::Error << "could not switch to new state: " << next_state_name << Logger::endl;
            diag_ss_rt_.addBehaviorSwitch(state_graphs_[current_state_->getStateName()], now, (err_cond?DiagBehaviorSwitch::ERROR:DiagBehaviorSwitch::STOP), predicate_list_);
            diag_bs_sync_.Set(diag_ss_rt_);
            error();
            return;
        }
        diag_ss_rt_.addBehaviorSwitch(state_graphs_[current_state_->getStateName()], now, (err_cond?DiagBehaviorSwitch::ERROR:DiagBehaviorSwitch::STOP), predicate_list_);
        diag_bs_sync_.Set(diag_ss_rt_);

        behavior_switch = true;
    }
    ros::Time time3 = rtt_rosclock::rtt_wall_now();

    //
    // if the behavior has changed, reorganize the graph
    //

    if (behavior_switch) {

        int new_behavior_idx = state_graphs_[current_state_->getStateName()];
        if (!switchToConfiguration_(new_behavior_idx)) {
            Logger::log() << Logger::Error << "could not switch graph configuration" << Logger::endl;
            error();
            return;
        }
    }
    ros::Time time4 = rtt_rosclock::rtt_wall_now();

    if (scheme_->getTaskState() != RTT::TaskContext::Running) {
        RTT::log(RTT::Error) << "Component is not in the running state: " << scheme_->getName() << RTT::endlog();
        error();
        return;
    }

    ros::Time time5 = rtt_rosclock::rtt_wall_now();

    RTT::os::TimeService::nsecs time_1 = RTT::os::TimeService::Instance()->getNSecs();
    scheme_->update();
    RTT::os::TimeService::nsecs time_2 = RTT::os::TimeService::Instance()->getNSecs();

    scheme_time_ = RTT::nsecs_to_Seconds(time_2) - RTT::nsecs_to_Seconds(time_1);

    // iterationEnd callback can be used by e.g. Gazebo simulator
    master_service_->iterationEnd();

    ros::Time time6 = rtt_rosclock::rtt_wall_now();

    double interval1 = (time2-time1).toSec();
    double interval2 = (time3-time2).toSec();
    double interval3 = (time4-time3).toSec();
    double interval4 = (time5-time4).toSec();
    double interval5 = (time6-time5).toSec();

    if (interval1 > interval1_ || counter_%5000 == 0) {
        interval1_ = interval1;
    }

    if (interval2 > interval2_ || counter_%5000 == 0) {
        interval2_ = interval2;
    }

    if (interval3 > interval3_ || counter_%5000 == 0) {
        interval3_ = interval3;
    }

    if (interval4 > interval4_ || counter_%5000 == 0) {
        interval4_ = interval4;
    }

    if (interval5 > interval5_ || counter_%5000 == 0) {
        interval5_ = interval5;
    }
}

ORO_LIST_COMPONENT_TYPE(MasterComponent)

ORO_CREATE_COMPONENT_LIBRARY()

