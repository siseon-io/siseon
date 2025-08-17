package siseon.backend.dto;

import jakarta.validation.constraints.NotNull;
import lombok.*;

import java.time.LocalDate;
import java.util.Map;

@Getter
@Setter
@NoArgsConstructor
@AllArgsConstructor
public class ProfileCreateRequest {

    @NotNull
    private String name;

    @NotNull
    private LocalDate birthDate;

    @NotNull
    private Integer height;

    @NotNull
    private Float leftVision;

    @NotNull
    private Float rightVision;

    private String imageUrl;

    private Map<String, Object> settings;
}