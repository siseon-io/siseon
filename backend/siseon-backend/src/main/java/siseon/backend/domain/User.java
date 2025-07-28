package siseon.backend.domain;

import jakarta.persistence.*;
import lombok.*;
import org.hibernate.annotations.CreationTimestamp;
import org.hibernate.annotations.UpdateTimestamp;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;


@Entity
@Table(name = "users")
@Getter @Setter
@NoArgsConstructor @AllArgsConstructor
@Builder
public class User {

    @Id @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "user_id")
    private Long id;

    @Column(nullable = false, length = 100, unique = true)
    private String email;

    @Column(nullable = false, length = 100)
    private String name;

    @Column(name = "picture_url")
    private String pictureUrl;

    @CreationTimestamp
    @Column(name = "created_at", nullable = false, updatable = false)
    private LocalDateTime createdAt;

    @UpdateTimestamp
    @Column(name = "updated_at", nullable = false)
    private LocalDateTime updatedAt;

    @OneToMany(mappedBy = "user", cascade = CascadeType.ALL, orphanRemoval = true)
    private List<Profile> profiles = new ArrayList<>();
}
