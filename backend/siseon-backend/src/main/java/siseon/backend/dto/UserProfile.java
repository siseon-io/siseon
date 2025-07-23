package siseon.backend.dto;

import java.time.LocalDateTime;

public record UserProfile(
        Long id,
        String email,
        String name,
        String pictureUrl,
        LocalDateTime createdAt,
        LocalDateTime updatedAt
) {}
