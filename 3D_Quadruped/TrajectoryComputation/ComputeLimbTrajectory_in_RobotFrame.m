for ResKnotIdx = 1:length(theta_result)
    RTheta_k_result = EulerAngle_to_RotationMatrix(phi_result, theta_result, psi_result, ResKnotIdx);
    r_result_k = [x_result(ResKnotIdx);y_result(ResKnotIdx);z_result(ResKnotIdx)]; %torso result at times step KnotIdx
    
    Plf_result_k = [Plfx_result(ResKnotIdx);Plfy_result(ResKnotIdx);Plfz_result(ResKnotIdx)];
    Plh_result_k = [Plhx_result(ResKnotIdx);Plhy_result(ResKnotIdx);Plhz_result(ResKnotIdx)];
    Prf_result_k = [Prfx_result(ResKnotIdx);Prfy_result(ResKnotIdx);Prfz_result(ResKnotIdx)];
    Prh_result_k = [Prhx_result(ResKnotIdx);Prhy_result(ResKnotIdx);Prhz_result(ResKnotIdx)];
    
    PlfBase_temp = RTheta_k_result'*(Plf_result_k-r_result_k);
    PlhBase_temp = RTheta_k_result'*(Plh_result_k-r_result_k);
    PrfBase_temp = RTheta_k_result'*(Prf_result_k-r_result_k);
    PrhBase_temp = RTheta_k_result'*(Prh_result_k-r_result_k);
    
    PlfxBase(ResKnotIdx) = PlfBase_temp(1);
    PlfyBase(ResKnotIdx) = PlfBase_temp(2);
    PlfzBase(ResKnotIdx) = PlfBase_temp(3);
    
    PlhxBase(ResKnotIdx) = PlhBase_temp(1);
    PlhyBase(ResKnotIdx) = PlhBase_temp(2);
    PlhzBase(ResKnotIdx) = PlhBase_temp(3);
    
    PrfxBase(ResKnotIdx) = PrfBase_temp(1);
    PrfyBase(ResKnotIdx) = PrfBase_temp(2);
    PrfzBase(ResKnotIdx) = PrfBase_temp(3);
    
    PrhxBase(ResKnotIdx) = PrhBase_temp(1);
    PrhyBase(ResKnotIdx) = PrhBase_temp(2);
    PrhzBase(ResKnotIdx) = PrhBase_temp(3);
    
end
