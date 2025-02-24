package com.example.controlserver.Models;

import org.springframework.data.annotation.Id;
import org.springframework.data.mongodb.core.mapping.DBRef;
import org.springframework.data.mongodb.core.mapping.Document;

import com.example.controlserver.Misc.JobStatus;

@Document(collection = "navigation_requests")
public class NavigationRequest {
    
    @Id
    private String id;

    @DBRef
    private UGV ugv;

    private Coordinate targetPose;

    private JobStatus jobStatus = JobStatus.CREATED;

    private String callbackURL;

    private String comment;

    public String getId() {
        return this.id;
    }

    public UGV getUgv() {
        return ugv;
    }

    public void setUgv(UGV ugv) {
        this.ugv = ugv;
    }

    public Coordinate getTargetPose() {
        return targetPose;
    }

    public void setTargetPose(Coordinate targetPose) {
        this.targetPose = targetPose;
    }

    public JobStatus getJobStatus() {
        return jobStatus;
    }

    public void setJobStatus(JobStatus jobStatus) {
        this.jobStatus = jobStatus;
    }

    public String getCallbackURL() {
        return callbackURL;
    }

    public void setCallbackURL(String callbackURL) {
        this.callbackURL = callbackURL;
    }

    public void setComment(String comment) {
        this.comment = comment;
    }

    public String getComment() {
        return this.comment;
    }

}
