package com.sprout.db_paisa;

public class DBPaisaScanResult {
    private boolean manifestVerificationPassed;
    private boolean announcementVerificationPassed;
    private boolean attestationResultPassed;
    private String attestationTimestamp;
    private String deviceID;
    private String deviceType;
    private String manufacturer;
    private String status;
    private String sensorsDescription;
    private String actuatorsDescription;
    private String networkDescription;
    private String description;

    private boolean expanded;

    public DBPaisaScanResult(boolean manifestVerificationPassed,
                             boolean announcementVerificationPassed, boolean attestationResultPassed,
                             String attestationTimestamp, String deviceID, String deviceType,
                             String manufacturer, String status, String sensorsDescription, String actuatorsDescription,
                             String networkDescription, String description) {
        this.manifestVerificationPassed = manifestVerificationPassed;
        this.announcementVerificationPassed = announcementVerificationPassed;
        this.attestationResultPassed = attestationResultPassed;
        this.attestationTimestamp = attestationTimestamp;
        this.deviceID = deviceID;
        this.deviceType = deviceType;
        this.manufacturer = manufacturer;
        this.status = status;
        this.sensorsDescription = sensorsDescription;
        this.actuatorsDescription = actuatorsDescription;
        this.networkDescription = networkDescription;
        this.description = description;
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Manifest Verification: " + (manifestVerificationPassed==true?"PASS":"FAIL") + "\n");
        sb.append("Response Verification: " + (announcementVerificationPassed==true?"PASS":"FAIL") + "\n");
        sb.append("Attestation Result: " + (attestationResultPassed==true?"PASS":"FAIL") + "\n\t\t\t\t\t Attested before "+attestationTimestamp+ "sec \n\n");
        sb.append("Device ID: " + deviceID+"\n");
        sb.append("Type: " + deviceType+"\n");
        sb.append("Status: " + status+"\n");
        sb.append("Sensors: "+ sensorsDescription + "\n");
        sb.append("Actuators: " + actuatorsDescription+"\n");
        sb.append("Network: " + networkDescription+"\n");
        sb.append("Description: " + description +"\n");
        return sb.toString();
    }

    public boolean isManifestVerificationPassed() {
        return manifestVerificationPassed;
    }

    public boolean isAnnouncementVerificationPassed() {
        return announcementVerificationPassed;
    }

    public boolean isAttestationResultPassed() {
        return attestationResultPassed;
    }

    public String getAttestationTimestamp() {
        return attestationTimestamp;
    }

    public String getDeviceID() {
        return deviceID;
    }

    public String getDeviceType() {
        return deviceType;
    }

    public String getManufacturer() {
        return manufacturer;
    }

    public String getNetworkDescription() {
        return networkDescription;
    }

    public String getDescription() {
        return description;
    }

    public String getStatus() {
        return status;
    }

    public String getActuatorsDescription() {
        return actuatorsDescription;
    }

    public boolean isExpanded() {
        return expanded;
    }

    public void setExpanded(boolean expanded) {
        this.expanded = expanded;
    }
}
