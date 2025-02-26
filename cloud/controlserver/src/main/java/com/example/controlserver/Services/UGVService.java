package com.example.controlserver.Services;

import com.example.controlserver.Models.UGV;
import com.example.controlserver.Misc.UGVStatus;
import com.example.controlserver.Repositories.UGVRepository;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
public class UGVService {

    @Autowired
    private UGVRepository ugvRepository;

    public UGV saveUGV(UGV ugv) {
        return ugvRepository.save(ugv);
    }

    public List<UGV> getAllUGVs() {
        return ugvRepository.findAll();
    }

    public UGV getUGVById(String id) {
        return ugvRepository.findById(id).orElseThrow(() -> new RuntimeException("UGV not found"));
    }

    public UGV updateUGV(String id, UGV ugvDetails) {
        UGV ugv = getUGVById(id);
        ugv.setName(ugvDetails.getName());
        ugv.setType(ugvDetails.getType());
        ugv.setStatus(ugvDetails.getStatus());
        return ugvRepository.save(ugv);
    }

    public void deleteUGV(String id) {
        ugvRepository.deleteById(id);
    }

    public UGV setUGVSessionID(String id, String sessionId) {
        UGV ugv = getUGVById(id);
        ugv.setSessionId(sessionId);
        return ugvRepository.save(ugv);
    }

    public boolean isUGVIDValid(String UGVId) {

        if(this.getUGVById(UGVId)!=null) 
            return true;

        return false;
    }

    public void saveUGVSession(String UGVId, String sessionId) {

        UGV ugv = this.getUGVById(UGVId);

        ugv.setSessionId(sessionId);

        this.updateUGV(UGVId, ugv);

    }

    public boolean checkUGVStatus(String UGVId, UGVStatus status) {

        if(this.getUGVById(UGVId).getStatus() != status) return false;
        return true;

    }

}
