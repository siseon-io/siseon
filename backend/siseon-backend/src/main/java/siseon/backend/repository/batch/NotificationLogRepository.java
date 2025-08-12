package siseon.backend.repository.batch;

import org.springframework.data.domain.PageRequest;
import org.springframework.data.jpa.repository.*;
import org.springframework.data.repository.query.Param;
import siseon.backend.domain.batch.NotificationLog;

import java.util.List;

public interface NotificationLogRepository extends JpaRepository<NotificationLog, Long> {

    @Query("""
      SELECT n FROM NotificationLog n
      WHERE n.profileId = :pid AND n.type = :type
      ORDER BY n.sentAt DESC
    """)
    List<NotificationLog> findRecent(@Param("pid") Long pid,
                                     @Param("type") String type,
                                     PageRequest page);
}