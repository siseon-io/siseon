package siseon.backend.dto;

public record JwtResponse(
        String accessToken,
        String refreshToken
) {}
