package com.example.controlserver.Schedulers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;

import com.example.controlserver.Misc.JobStatus;
import com.example.controlserver.Misc.UGVStatus;
import com.example.controlserver.Models.NavigationRequest;
import com.example.controlserver.Models.UGV;
import com.example.controlserver.Services.NavigationRequestService;
import com.example.controlserver.Services.SocketConnectionHandler;
import com.example.controlserver.Services.UGVService;

@Component
public class NavigationRequestScheduler {

    @Autowired
    private NavigationRequestService navigationRequestService;

    @Autowired
    private UGVService ugvService;

    @Autowired
    private SocketConnectionHandler socketConnectionHandler;

    private static final Logger logger = LoggerFactory.getLogger(NavigationRequestScheduler.class);
    
    @Scheduled(fixedRate = 10000) 
    public void executePendingNavigationRequests() {

        logger.info("Started navigation request scheduler job");

        // Pick all jobs which are not completed in the job queue
        List<NavigationRequest> pendingJobs = navigationRequestService.getPendingJobs();
        
        // Start each job one by one and mark it as started
        for(NavigationRequest navReq : pendingJobs) {
            UGV ugv = navReq.getUgv();
            if(ugv.getStatus()!=UGVStatus.ONLINE) {
                navReq.setJobStatus(JobStatus.FAILED);
                navReq.setComment("UGV not online");
                logger.error("Failed to start navigation job "+navReq.getId()+" :: UGV not online");
                navigationRequestService.updateNavigationRequest(navReq.getId(), navReq);
            } else {
                try {
                    Map<String, Object> ugvRequest = new HashMap<>();
                    ugvRequest.put("request_type", "navigate");
                    ugvRequest.put("coordinates", navReq.getTargetPose());
                    socketConnectionHandler.sendMessageToClient(ugv.getSessionId(), new TextMessage(ugvRequest.toString()));
                    logger.info("Started navigation job with ID : "+navReq.getId());
                    ugv.setStatus(UGVStatus.BUSY);
                    navReq.setJobStatus(JobStatus.STARTED);
                } catch (Exception e) {
                    ugv.setStatus(UGVStatus.ONLINE);
                    navReq.setJobStatus(JobStatus.FAILED);
                    navReq.setComment(e.getMessage());;
                    logger.error("Failed to start navigation job "+navReq.getId()+" :: "+e.getMessage());
                    e.printStackTrace();
                } finally {
                    navigationRequestService.updateNavigationRequest(navReq.getId(), navReq);
                    ugvService.updateUGV(ugv.getId(), ugv);
                }
            }
        }
        // End
    }
}
