/*
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

 */

#ifndef HUB_EFFECTOR_H
#define HUB_EFFECTOR_H

#include <Eigen/Dense>
#include "simulation/dynamics/_GeneralModuleFiles/stateEffector.h"
#include "simulation/dynamics/_GeneralModuleFiles/stateData.h"
#include "architecture/utilities/avsEigenMRP.h"
#include "architecture/utilities/bskLogging.h"

/*! @brief spaecraft hub effector class */
class HubEffector : public StateEffector {
public:
    double mHub;                         //!< [kg] mass of the hub
    uint64_t MRPSwitchCount;             //!< -- Count on times we've shadowed
    std::string nameOfHubPositionRN;     //!< -- Identifier for reference position states
    std::string nameOfHubPositionBR;     //!< -- Identifier for hub position relative to the reference
    std::string nameOfHubVelocityRN;     //!< -- Identifier for reference position states
    std::string nameOfHubVelocityBR;     //!< -- Identifier for hub position relative to the reference
    std::string nameOfHubSigma;          //!< -- Identifier for hub sigmaBN states
    std::string nameOfHubOmega;          //!< -- Identifier for hub omegaBN_B states
    Eigen::Vector3d r_BcB_B;             //!< [m] vector from point B to CoM of hub in B frame components
    Eigen::Matrix3d IHubPntBc_B;         //!< [kg m^2] Inertia of hub about point Bc in B frame components
    BackSubMatrices hubBackSubMatrices;  //!< class method
    Eigen::Vector3d sigma_BNInit;        //!< -- Initial attitude of the spacecraft wrt base
    Eigen::Vector3d omega_BN_BInit;      //!< [r/s] Initial attitude rate of the spacecraf wrt base
    BSKLogger bskLogger;                      //!< -- BSK Logging

public:
    HubEffector();                       //!< -- Contructor
    ~HubEffector();                      //!< -- Destructor
    void linkInStates(DynParamManager& statesIn);  //!< -- Method to give the hub access to states
    void registerStates(DynParamManager& states);  //!< -- Method for the hub to register some states
    void updateEffectorMassProps(double integTime);  //!< -- Method for the hub to update its mass props for the s/c
    void computeDerivatives(double integTime, Eigen::Vector3d rDDot_BN_N, Eigen::Vector3d omegaDot_BN_B, Eigen::Vector3d sigma_BN);  //!< -- Method for the hub to compute it's derivatives
    void updateEnergyMomContributions(double integTime, Eigen::Vector3d & rotAngMomPntCContr_B,
                                      double & rotEnergyContr, Eigen::Vector3d omega_BN_B); //!< -- Add contributions to energy and momentum
    void modifyStates(double integTime); //!< -- Method to switch MRPs
    void prependSpacecraftNameToStates(); //!< class method

private:
    Eigen::Vector3d r_BcP_P;             //!< [m] vector from point B to CoM of hub in B frame components
    Eigen::Matrix3d IHubPntBc_P;         //!< [kg m^2] Inertia of hub about point Bc in B frame components
    StateData *positionState_RN;         //!< [m] state data container for reference position
    StateData *velocityState_RN;         //!< [m/s] state data container for reference velocity
    StateData *positionState_BR;         //!< [m] state data container for B position wrt the non-rotating reference point
    StateData *velocityState_BR;         //!< [m/s] state data container for B velocity wrt the non-rotating reference point
    StateData *sigmaState;               //!< [-] State data container for hub sigma_BN
    StateData *omegaState;               //!< [-] State data container for hub omegaBN_B
};

#endif /* HUB_EFFECTOR_H */
