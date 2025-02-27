package com.example.controlserver.Schedulers;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import com.example.controlserver.Misc.UGVStatus;
import com.example.controlserver.Models.UGV;
import com.example.controlserver.Services.SocketConnectionHandler;
import com.example.controlserver.Services.UGVService;

@Component
public class UGVHeartbeatScheduler {

    @Autowired
    private UGVService ugvService;

    @Autowired
    private SocketConnectionHandler socketConnectionHandler;

    private static final Logger logger = LoggerFactory.getLogger(UGVHeartbeatScheduler.class);

    @Scheduled(fixedDelay = 10000)
    public void broadcastUGVHeartBeat() {
        logger.info("Started UGV heartbeat request broadcaster");

        List<UGV> ugvList = ugvService.getAllUGVs();

        Map<String, Object> request = new HashMap<>();
        request.put("request_type", "heartbeat");
        for(UGV ugv : ugvList) {
            try {
                ugv.setStatus(UGVStatus.OFFLINE);
                socketConnectionHandler.sendMessageToClient(ugv.getSessionId(), request);
                ugvService.updateUGV(ugv.getId(), ugv);
            } catch (Exception e) {
                logger.error("Error sending broadcase to UGV "+ugv.getId());
            }
        }
    }
}
