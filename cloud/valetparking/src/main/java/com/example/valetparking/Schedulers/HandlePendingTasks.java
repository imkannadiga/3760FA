package com.example.valetparking.Schedulers;

import java.util.List;
import java.util.Map;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import com.example.valetparking.Helpers.ControlHelper;
import com.example.valetparking.Helpers.DropOffSpotHelper;
import com.example.valetparking.Helpers.ParkingHelper;
import com.example.valetparking.Helpers.Phase;
import com.example.valetparking.Models.Requests;
import com.example.valetparking.Models.UGVStatus;
import com.example.valetparking.Repositories.RequestRepository;
import com.example.valetparking.Repositories.UGVStatusRepository;

@Component
public class HandlePendingTasks {

    @Autowired
    RequestRepository requestRepository;

    @Autowired
    ParkingHelper parkingHelper;

    @Autowired
    UGVStatusRepository ugvStatusRepository;

    @Autowired
    DropOffSpotHelper dropOffSpotHelper;

    @Autowired
    ControlHelper controlHelper;

    private static final Logger logger = LoggerFactory.getLogger(HandlePendingTasks.class);


    @Scheduled(fixedDelay = 10000)
    public void handleNextStep() {
        logger.info("Started handling pending tasks scheduler");
        List<Requests> requests = requestRepository.findByStepTwoPending(true);
        for (Requests req : requests) {
            switch (req.getCurrentTask()) {
                case PARK:
                    switch (req.getCurrentPhase()) {
                        case DROP_OFF_LOCATION:
                            // update currentPhase to PARKED
                            req.setCurrentPhase(Phase.PARKED);
                            // Parking subroutine
                            Map<String, Object> parkingSpot = parkingHelper.getAndBlockAvailableParkingSpot();
                            if(parkingSpot==null) break;
                            dropOffSpotHelper.releaseSpot(req.getUgvId());
                            String newNavId = controlHelper.sendRequestToControlServer(req.getUgvId(), parkingSpot);
                            req.setCurrentJobId(newNavId);
                            req.setParkingSpotId((String) parkingSpot.get("id"));
                            req.setStepTwoPending(false);
                            requestRepository.save(req);
                            break;

                        case PARKED:
                            UGVStatus status = ugvStatusRepository.findByUgvId(req.getUgvId()).get(0);
                            status.setCurrentPhase(Phase.PARKED);
                            ugvStatusRepository.save(status);
                            break;

                        default:
                            break;
                    }
                    break;

                case RETRIEVE:
                    switch (req.getCurrentPhase()) {
                        case DROP_OFF_LOCATION:
                            req.setCurrentPhase(Phase.DRIVE_AWAY_LOCATION);
                            // Drive away subroutine:TODO
                            req.setStepTwoPending(false);
                            requestRepository.save(req);
                            break;

                        case DRIVE_AWAY_LOCATION:
                            UGVStatus status = ugvStatusRepository.findByUgvId(req.getUgvId()).get(0);
                            status.setCurrentPhase(Phase.DRIVE_AWAY_LOCATION);
                            ugvStatusRepository.save(status);
                            break;

                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
        }
    }
}
