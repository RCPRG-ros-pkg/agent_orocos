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

#include "subsystem_common/input_data.h"
#include "subsystem_common/master_service_requester.h"
#include "subsystem_common/master_service.h"

using namespace RTT;

class DiagStateSwitch {
public:
    enum Reason {INVALID, INIT, STOP, ERROR};
    int prev_id_;
    int id_;
    RTT::os::TimeService::nsecs time_;
    Reason reason_;
    subsystem_common::PredicateListPtr pred_;

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

class DiagStateSwitchHistory {
public:
    DiagStateSwitchHistory()
        : idx_(0)
    {}

    void addStateSwitch(int new_state_id, RTT::os::TimeService::nsecs time, DiagStateSwitch::Reason reason, subsystem_common::PredicateListConstPtr pred = subsystem_common::PredicateListConstPtr()) {
        if (h_.size() == 0) {
            return;
        }
        if (new_state_id < 0 || new_state_id >= states_count_) {
            Logger::log() << Logger::Error << "addStateSwitch: wrong state id: " << new_state_id << ", should be in range [0, " << (states_count_-1) << "]" << Logger::endl;
        }
        int prev_idx = (idx_+h_.size()-1)%h_.size();
        int prev_state_id = h_[prev_idx].id_;
        h_[idx_].id_ = new_state_id;
        h_[idx_].prev_id_ = prev_state_id;
        h_[idx_].time_ = time;
        h_[idx_].reason_ = reason;
        if (pred) {
            *(h_[idx_].pred_) = *pred;
        }
        idx_ = (idx_+1) % h_.size();

        if (prev_state_id >= 0) {
            for (int i = 0; i < st_.size(); ++i) {
                if ( (st_[i].id_ == new_state_id && st_[i].prev_id_ == prev_state_id) ||
                    (st_[i].id_ == -1))
                {
                    st_[i].id_ = new_state_id;
                    st_[i].prev_id_ = prev_state_id;
                    st_[i].time_ = time;
                    st_[i].reason_ = reason;
                    if (pred) {
                        *(st_[i].pred_) = *pred;
                    }
                    break;
                }
            }
        }
    }

    void setSize(size_t size, size_t states_count, RTT::OperationCaller<subsystem_common::PredicateListPtr()> subsystem_common::MasterServiceRequester::*func, subsystem_common::MasterServiceRequester& a) {
        h_.resize(size);
        for (int i = 0; i < h_.size(); ++i) {
            h_[i].id_ = -1;
            h_[i].reason_ = DiagStateSwitch::INVALID;
            h_[i].pred_ = (a.*func)();
        }
        states_count_ = states_count;
        st_.resize(states_count*states_count);
        for (int i = 0; i < st_.size(); ++i) {
            st_[i].id_ = -1;
            st_[i].reason_ = DiagStateSwitch::INVALID;
            st_[i].pred_ = (a.*func)();
        }
        idx_ = 0;
    }

    bool getStateSwitchHistory(int idx, DiagStateSwitch &ss) const {
        if (idx < 0 || idx >= h_.size()) {
            return false;
        }
        int i = (idx_-idx-1+h_.size()*2) % h_.size();
        if (h_[i].reason_ == DiagStateSwitch::INVALID) {
            return false;
        }
        ss = h_[i];
        return true;
    }

    bool getStateSwitchStateList(int idx, DiagStateSwitch &ss) const {
        if (idx < 0 || idx >= st_.size()) {
            return false;
        }
        if (st_[idx].reason_ == DiagStateSwitch::INVALID) {
            return false;
        }
        ss = st_[idx];
        return true;
    }

private:

    std::vector<DiagStateSwitch> h_;
    std::vector<DiagStateSwitch> st_;
    int idx_, states_count_;
};

class MasterComponent: public RTT::TaskContext {
public:
    explicit MasterComponent(const std::string &name);

    bool configureHook();

    void cleanupHook();

    bool startHook();

    void stopHook();

    void updateHook();

    std::string getDiag();

    bool addConmanScheme(RTT::TaskContext* scheme);

    void setThreadName(const std::string& thread_name);

private:
    void calculateConflictingComponents();
    bool isGraphOk() const;

    int current_state_id_;
    std::map<std::string, int > state_graphs_;

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

    RTT::base::DataObjectLockFree<DiagStateSwitchHistory > diag_bs_sync_;
    DiagStateSwitchHistory diag_ss_rt_;

    boost::shared_ptr<subsystem_common::MasterServiceRequester > master_service_;

    boost::shared_ptr<subsystem_common::InputData > in_data_;

    subsystem_common::PredicateListPtr predicate_list_;

    RTT::Seconds last_exec_time_, last_exec_period_;
    RTT::os::TimeService::nsecs last_update_time_;
    RTT::os::TimeService::Seconds scheme_time_;

    int state_switch_history_length_;

    std::set<std::pair<std::string, std::string > > conflicting_components_;

    int counter_;

    double interval1_;
    double interval2_;
    double interval3_;
    double interval4_;
    double interval5_;

    std::string thread_name_;

    double read_buffer_timeout_;
    bool use_sim_time_;
    ros::Time time_last_s_;

    std::vector<unsigned int > period_histogram_;
    std::vector<float > period_histogram_ranges_;
};

void MasterComponent::setThreadName(const std::string& thread_name) {
    thread_name_ = thread_name;
    Logger::log() << Logger::Info << "master component thread name: " << thread_name_ << Logger::endl;
}

MasterComponent::MasterComponent(const std::string &name)
    : TaskContext(name, PreOperational)
    , state_switch_history_length_(5)
    , scheme_time_(0)
    , use_sim_time_(false)
{
    this->addOperation("getDiag", &MasterComponent::getDiag, this, RTT::ClientThread);

    this->addOperation("addConmanScheme", &MasterComponent::addConmanScheme, this, RTT::ClientThread);

    this->addOperation("setThreadName", &MasterComponent::setThreadName, this, RTT::ClientThread);

    addProperty("state_switch_history_length", state_switch_history_length_);

    addProperty("use_sim_time", use_sim_time_);
}

std::string MasterComponent::getDiag() {
    std::ostringstream strs;

    strs << "<mcd>";

    // write states history
    strs << "<h>";

    DiagStateSwitchHistory ss;
    diag_bs_sync_.Get(ss);

    for (int i = 0; ; ++i) {
        DiagStateSwitch s;
        if (!ss.getStateSwitchHistory(i, s)) {
            break;
        }
        RTT::os::TimeService::Seconds switch_interval = RTT::nsecs_to_Seconds(last_update_time_ - s.time_);

        std::string err_str;
        if (s.pred_) {
            err_str = master_service_->getPredicatesStr(s.pred_);
        }

        std::string state_name, prev_state_name;
        if (s.id_ >= 0) {
            state_name = master_service_->getStates()[s.id_]->getName();
            if (s.prev_id_ >= 0) {
                prev_state_name = master_service_->getStates()[s.prev_id_]->getName();
            }
            else {
                prev_state_name = "";
            }
        }
        else {
            state_name = "INV_BEH";
        }
        strs << "<ss n=\"" << state_name << "\" p=\"" << prev_state_name << "\" r=\""
             << s.getReasonStr() << "\" t=\"" << switch_interval << "\" e=\""
             << err_str << "\" />";
    }
    strs << "</h>";

    // write information about state switch for each state
    strs << "<si>";

    for (int i = 0;; ++i) {
        DiagStateSwitch s;
        if (!ss.getStateSwitchStateList(i, s)) {
            break;
        }
        RTT::os::TimeService::Seconds switch_interval = RTT::nsecs_to_Seconds(last_update_time_ - s.time_);

        std::string err_str;
        if (s.pred_) {
            err_str = master_service_->getPredicatesStr(s.pred_);
        }

        std::string state_name, prev_state_name;
        if (s.id_ >= 0 && s.prev_id_ >= 0) {
            state_name = master_service_->getStates()[s.id_]->getName();
            prev_state_name = master_service_->getStates()[s.prev_id_]->getName();
        }
        else {
            state_name = "INV_ST";
            prev_state_name = "INV_ST";
        }
        strs << "<ss n=\"" << state_name << "\" p=\"" << prev_state_name << "\" r=\""
             << s.getReasonStr() << "\" t=\"" << switch_interval << "\" e=\""
             << err_str << "\" />";
    }
    strs << "</si>";

    // write predicate list
    strs << "<pr v=\"" << master_service_->getPredicatesStr(predicate_list_) << "\" />";

    // write period
    strs << "<p>" << last_exec_period_ << "</p>";
    strs << "<t_tf>" << scheme_time_ << "</t_tf>";

    strs << "<int1>" << interval1_ << "</int1>";
    strs << "<int2>" << interval2_ << "</int2>";
    strs << "<int3>" << interval3_ << "</int3>";
    strs << "<int4>" << interval4_ << "</int4>";
    strs << "<int5>" << interval5_ << "</int5>";

    // write histogram of period
    strs << "<ph>";
    for (int i=0; i < period_histogram_.size(); ++i) {
        strs << period_histogram_[i] << " ";
    }
    strs << "</ph>";

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
                Logger::log() << Logger::Info << "Conflicting components for input port " << scheme_peers_[i]->getName() << "." << comp_ports[j] << ":" << Logger::endl;
                for (int k = 0; k < comp_out.size(); ++k) {
                    Logger::log() << Logger::Info << "    \'" << comp_out[k] << "\' (port: \'" << port_out[k] << "\')" << Logger::endl;
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
    Logger::log() << Logger::Info << "Conflicting components pairs:" << Logger::endl;
    for (std::set<std::pair<std::string, std::string > >::const_iterator it = conflicting_components_.begin(); it != conflicting_components_.end(); ++it) {
        Logger::log() << Logger::Info << "    " << it->first << ", " << it->second << Logger::endl;
    }
}

bool MasterComponent::configureHook() {
    Logger::In in("MasterComponent::configureHook");

    master_service_ = this->getProvider<subsystem_common::MasterServiceRequester >("master");
    if (!master_service_) {
        Logger::log() << Logger::Error << "Unable to load subsystem_common::MasterService" << Logger::endl;
        return false;
    }

    if (!master_service_->configureBuffers()) {
        Logger::log() << Logger::Error << "could not configure subsystem buffers" << Logger::endl;
        return false;
    }

    predicate_list_ = master_service_->allocatePredicateList();
    if (!predicate_list_) {
        Logger::log() << Logger::Error << "could not allocate predicate list" << Logger::endl;
        return false;
    }

    int initial_state_index = master_service_->getInitialStateIndex();
    const std::vector<const subsystem_common::StateBase* >& states = master_service_->getStates();
    const std::vector<const subsystem_common::BehaviorBase* >& behaviors = master_service_->getBehaviors();

    Logger::log() << Logger::Info << "initial state idx: " << master_service_->getInitialStateIndex() << ", name: " << states[initial_state_index]->getName() << Logger::endl;

    current_state_id_ = initial_state_index;

    // retrieve the vector of peers of conman scheme
    TaskContext::PeerList scheme_peers_names = scheme_->getPeerList();
    for (int pi = 0; pi < scheme_peers_names.size(); ++pi) {
        scheme_peers_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
        scheme_peers_const_.push_back( scheme_->getPeer(scheme_peers_names[pi]) );
    }

    diag_ss_rt_.setSize(state_switch_history_length_, states.size(), &subsystem_common::MasterServiceRequester::allocatePredicateList, *master_service_);

    diag_bs_sync_.data_sample(diag_ss_rt_);
    diag_bs_sync_.Set(diag_ss_rt_);

    RTT::OperationInterfacePart *hasBlockOp = scheme_->getOperation("hasBlock");
    if (hasBlockOp == NULL) {
        Logger::log() << Logger::Error << "the peer " << scheme_->getName() << " has no matching operation hasBlock" << Logger::endl;
        return false;
    }

    hasBlock_ =  RTT::OperationCaller<bool(const std::string &)>(
        hasBlockOp, scheme_->engine());

    std::vector<std::set<std::string > > running_components_in_state;
    // get names of all components that are needed for all behaviors
    for (int i = 0; i < states.size(); ++i) {
        std::set<std::string > rc_in_state;
        const std::vector<int >& behavior_indices = states[i]->getBehaviorIndices();
        for (int j = 0; j < behavior_indices.size(); ++j) {
            const std::vector<std::string >& running_components = behaviors[behavior_indices[j]]->getRunningComponents();
            rc_in_state.insert(running_components.begin(), running_components.end());            
        }
        for (std::set<std::string >::const_iterator it = rc_in_state.begin(); it != rc_in_state.end(); ++it) {
            if (hasBlock_( (*it) )) {
                switchable_components_.insert( (*it) );
            }
            else {
                Logger::log() << Logger::Error << "could not find a component \'" << (*it) << "\' in the scheme blocks list" << Logger::endl;
                return false;
            }
        }
        running_components_in_state.push_back(rc_in_state);
    }

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
    Logger::log() << Logger::Info << "conman graph configurations:" << Logger::endl;

    for (int i = 0; i < states.size(); ++i) {
        const std::vector<std::string > vec_running(running_components_in_state[i].begin(), running_components_in_state[i].end());

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

//        Logger::log() << Logger::Info << i << " '" << states_[i]->getStateName() << "':  s:[" << str_stopped << "], r:[" << str_running << "]" << Logger::endl;

        addGraphConfiguration_(i, vec_stopped, vec_running);
    }

    in_data_ = master_service_->allocateBuffersData();
    if (!in_data_) {
        Logger::log() << Logger::Error << "Unable to get InputData sample" << Logger::endl;
        return false;
    }

    // run this function for proper initialization of predicate functions
    master_service_->calculatePredicates(in_data_, scheme_peers_const_, predicate_list_);


    // prepare data structures for histogram of period time
    period_histogram_ranges_.push_back(0.0001);
    period_histogram_ranges_.push_back(0.0002);
    period_histogram_ranges_.push_back(0.0003);
    period_histogram_ranges_.push_back(0.0004);
    period_histogram_ranges_.push_back(0.0006);
    period_histogram_ranges_.push_back(0.0008);
    period_histogram_ranges_.push_back(0.001);
    period_histogram_ranges_.push_back(0.0012);
    period_histogram_ranges_.push_back(0.0014);
    period_histogram_ranges_.push_back(0.0016);
    period_histogram_ranges_.push_back(0.002);
    period_histogram_ranges_.push_back(0.0024);
    period_histogram_ranges_.push_back(0.0028);
    period_histogram_ranges_.push_back(0.0032);
    period_histogram_ranges_.push_back(0.0038);
    period_histogram_ranges_.push_back(0.0050);
    period_histogram_ranges_.push_back(0.0060);
    period_histogram_ranges_.push_back(0.0080);
    period_histogram_ranges_.push_back(0.01);
    period_histogram_ranges_.push_back(0.02);
    period_histogram_ranges_.push_back(0.03);
    period_histogram_ranges_.push_back(0.05);

    period_histogram_.resize(period_histogram_ranges_.size()+1, 0);

    return true;
}

bool MasterComponent::startHook() {
    first_step_ = true;
    return true;
}

void MasterComponent::cleanupHook() {
    master_service_->cleanupBuffers();
}

void MasterComponent::stopHook() {
}

bool MasterComponent::isGraphOk() const {

    int current_graph_id = current_state_id_;
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
    RTT::os::TimeService::Seconds time = RTT::nsecs_to_Seconds(now);
    //period = RTT::nsecs_to_Seconds(RTT::os::TimeService::Instance()->getNSecs(last_update_time_));

    ros::Time time1 = rtt_rosclock::rtt_wall_now();

    // Store update time
    last_update_time_ = now;

    // Compute statistics describing how often update is being called
    last_exec_period_ = time - last_exec_time_;
    last_exec_time_ = time;
    bool period_in_range = false;
    for (int i=0; i < period_histogram_ranges_.size(); ++i) {
        if (last_exec_period_ < period_histogram_ranges_[i]) {
            if (period_histogram_[i] >= 4294967295) {
                // clear the histogram
                for (int j=0; j < period_histogram_.size(); ++j) {
                    period_histogram_[j] = 0;
                }
            }
            ++period_histogram_[i];
            period_in_range = true;
            break;
        }
    }
    if (!period_in_range) {
        if (period_histogram_[period_histogram_.size()-1] >= 4294967295) {
            for (int j=0; j < period_histogram_.size(); ++j) {
                // clear the histogram
                period_histogram_[j] = 0;
            }
        }
        ++period_histogram_[period_histogram_.size()-1];
    }

    // iterationBegin callback can be used by e.g. Gazebo simulator
    master_service_->iterationBegin();

    master_service_->getBuffers(in_data_);
    counter_++;

    master_service_->writePorts(in_data_);

    bool state_switch = false;

    bool graphOk = true;
    if (first_step_) {
        first_step_ = false;
        state_switch = true;

        diag_ss_rt_.addStateSwitch(current_state_id_, now, DiagStateSwitch::INIT);
        diag_bs_sync_.Set(diag_ss_rt_);
    }
    else {
        // conman graph is initialized in first iteration
        graphOk = isGraphOk();
    }

    master_service_->calculatePredicates(in_data_, scheme_peers_const_, predicate_list_);
    predicate_list_->CURRENT_BEHAVIOR_OK = graphOk;

    const std::vector<const subsystem_common::StateBase* >& states = master_service_->getStates();
    const std::vector<const subsystem_common::BehaviorBase* >& behaviors = master_service_->getBehaviors();

    bool err_cond = false;
    const std::vector<int >& behavior_indices = states[current_state_id_]->getBehaviorIndices();
    for (int i = 0; i < behavior_indices.size(); ++i) {
        if (behaviors[behavior_indices[i]]->checkErrorCondition(predicate_list_)) {
            err_cond = true;
            break;
        }
    }

    bool stop_cond = false;

    if (!err_cond) {
        for (int i = 0; i < behavior_indices.size(); ++i) {
            if (behaviors[behavior_indices[i]]->checkStopCondition(predicate_list_)) {
                stop_cond = true;
                break;
            }
        }
    }

    predicate_list_->IN_ERROR = err_cond;
    ros::Time time2 = rtt_rosclock::rtt_wall_now();

    if (stop_cond || err_cond) {
        current_state_id_ = states[current_state_id_]->calculateNextState(predicate_list_);
        if (current_state_id_ < 0) {
            Logger::log() << Logger::Error << "could not switch to new state: " << current_state_id_ << Logger::endl;
            diag_ss_rt_.addStateSwitch(current_state_id_, now, (err_cond?DiagStateSwitch::ERROR:DiagStateSwitch::STOP), predicate_list_);
            diag_bs_sync_.Set(diag_ss_rt_);
            error();
            return;
        }

        diag_ss_rt_.addStateSwitch(current_state_id_, now, (err_cond?DiagStateSwitch::ERROR:DiagStateSwitch::STOP), predicate_list_);
        diag_bs_sync_.Set(diag_ss_rt_);

        state_switch = true;
    }
    ros::Time time3 = rtt_rosclock::rtt_wall_now();

    //
    // if the behavior has changed, reorganize the graph
    //

    if (state_switch) {
        if (use_sim_time_) {
            read_buffer_timeout_ = master_service_->getStateBufferGroup(current_state_id_).first_timeout_sim;
        }
        else {
            read_buffer_timeout_ = master_service_->getStateBufferGroup(current_state_id_).first_timeout;
        }

        if (!switchToConfiguration_(current_state_id_)) {
            Logger::log() << Logger::Error << "could not switch graph configuration" << Logger::endl;
            error();
            return;
        }
    }
    ros::Time time4 = rtt_rosclock::rtt_wall_now();

    if (scheme_->getTaskState() != RTT::TaskContext::Running) {
        Logger::log() << Logger::Error << "Component is not in the running state: " << scheme_->getName() << Logger::endl;
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

    while (true) {
        double timeout_s = master_service_->getStateBufferGroup(current_state_id_).min_period - (rtt_rosclock::rtt_now() - time_last_s_).toSec();  // calculate sleep time
        if (timeout_s <= 0) {
            break;
        }
        if (timeout_s < 0.0001) {
            timeout_s = 0.0001;
        }
        usleep( static_cast<int >((timeout_s)*1000000.0) );
    }
    time_last_s_ = rtt_rosclock::rtt_now();     // save time

    if (master_service_->bufferGroupRead( master_service_->getStateBufferGroup(current_state_id_).id, read_buffer_timeout_ )) {
        if (use_sim_time_) {
            read_buffer_timeout_ = master_service_->getStateBufferGroup(current_state_id_).first_timeout_sim;
        }
        else {
            read_buffer_timeout_ = master_service_->getStateBufferGroup(current_state_id_).first_timeout;
        }
    }
    else {
        read_buffer_timeout_ = master_service_->getStateBufferGroup(current_state_id_).next_timeout;
    }

    trigger();
}

ORO_LIST_COMPONENT_TYPE(MasterComponent)

ORO_CREATE_COMPONENT_LIBRARY()

