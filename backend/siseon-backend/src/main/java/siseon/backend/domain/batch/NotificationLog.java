package siseon.backend.domain.batch;

import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Entity
@Table(
        name = "notification_logs",
        indexes = {
                @Index(
                        name = "idx_profile_type_time",
                        columnList = "profile_id, type, sent_at"
                )
        }
)
@Getter @Setter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class NotificationLog {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "profile_id", nullable = false)
    private Long profileId;

    @Column(name = "type", nullable = false, length = 32)
    private String type;

    @Column(name = "sent_at", nullable = false)
    private LocalDateTime sentAt;
}