package siseon.backend.dto;

import lombok.Getter;
import siseon.backend.domain.main.Device;

@Getter
public class DeviceResponse {
    private Long deviceId;
    private Long profileId;
    private String serialNumber;

    public DeviceResponse(Device device) {
        this.deviceId     = device.getDeviceId();
        this.profileId    = device.getProfileId().getId();
        this.serialNumber = device.getSerialNumber();
    }
}
