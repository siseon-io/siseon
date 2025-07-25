package siseon.backend.dto;

import com.fasterxml.jackson.annotation.JsonProperty;

public record GoogleUserInfo(
        @JsonProperty("email") String email,
        @JsonProperty("name") String name,
        @JsonProperty("picture") String picture
) {}
