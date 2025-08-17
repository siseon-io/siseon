package siseon.backend.repository.main;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Modifying;
import org.springframework.data.jpa.repository.Query;
import org.springframework.data.repository.query.Param;
import org.springframework.stereotype.Repository;
import siseon.backend.domain.main.Profile;
import siseon.backend.domain.main.User;

import java.util.List;
import java.util.Optional;

@Repository
public interface ProfileRepository extends JpaRepository<Profile, Long> {

    List<Profile> findAllByUser(User user);

    Optional<Profile> findByFcmToken(String fcmToken);

    @Modifying
    @Query("UPDATE Profile p SET p.fcmToken = NULL " +
            "WHERE p.fcmToken = :fcmToken AND p.id <> :profileId")
    int clearSameTokenFromOthers(@Param("fcmToken") String fcmToken,
                                 @Param("profileId") Long profileId);
}