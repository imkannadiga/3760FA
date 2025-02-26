// DTO for creating a NavigationRequest
package com.example.controlserver.DTOs;

import java.util.Map;

public class CreateNavigationRequestDTO {

    private String ugvId;
    private Map<String, Object> targetPose;
    private String callbackURL;

    // Getters and Setters
    public String getUgvId() {
        return ugvId;
    }

    public void setUgvId(String ugvId) {
        this.ugvId = ugvId;
    }

    public Map<String, Object> getTargetPose() {
        return this.targetPose;
    }

    public void setTargetPose(Map<String, Object> targetPose) {
        this.targetPose = targetPose;
    }

    public String getCallbackURL() {
        return callbackURL;
    }

    public void setCallbackURL(String callbackURL) {
        this.callbackURL = callbackURL;
    }
}
