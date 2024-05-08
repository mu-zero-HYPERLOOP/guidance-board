#include "control_transition.hpp"


void airgap_transition::init(float airgap_start) {
    m_airgap_start = airgap_start;
    m_airgap_current = airgap_start;
}

void airgap_transition::start_transition(float target_airgap, float time) {
    if(!m_in_transition) {
        m_airgap_end = target_airgap;
        m_step_cycles = (int)(20000.0 * time / 101.0);
        m_in_transition = true;
    }
}

float airgap_transition::step() {
    if(m_in_transition) {
        m_transition_step_counter += 1;
        if(m_transition_step_counter > m_step_cycles) {
            m_transition_step_counter = 0;
            m_transition_counter += 1;
            
            if(m_transition_counter > 100) {
                // transition finished
                m_in_transition = false;
                m_airgap_start = m_airgap_end;
                m_transition_counter = 0;
                m_transition_step_counter = 0;
                m_airgap_current = m_airgap_end;
            }
            m_airgap_current = (m_airgap_start + (m_airgap_end - m_airgap_start) * sigmoid101[m_transition_counter]);
        }
    }
    return m_airgap_current;
}





bool airgap_transition::m_in_transition;
float airgap_transition::m_airgap_start;
float airgap_transition::m_airgap_current;
float airgap_transition::m_airgap_end;
float airgap_transition::m_transition_time;
int airgap_transition::m_transition_step_counter;
int airgap_transition::m_transition_counter;
int airgap_transition::m_step_cycles;