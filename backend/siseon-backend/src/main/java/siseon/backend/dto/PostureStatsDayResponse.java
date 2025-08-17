package siseon.backend.dto;

import lombok.*;
import lombok.experimental.SuperBuilder;

import java.time.LocalDate;

@Getter
@SuperBuilder
@AllArgsConstructor
@NoArgsConstructor
public class PostureStatsDayResponse extends PostureStatsResponse {

    private LocalDate statDate;
    private Integer goodCount;
    private Integer badCount;
    private Integer totalCount;
}