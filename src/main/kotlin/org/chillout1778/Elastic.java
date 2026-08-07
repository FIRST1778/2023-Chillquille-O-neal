package org.chillout1778;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/**
 * Utility class for interacting with the Elastic Dashboard over NetworkTables.
 */
public class Elastic {
    private static final NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    private static final StringPublisher selectedTabPub = elasticTable.getStringTopic("selectedTab").publish();
    private static final StringPublisher notificationPub = elasticTable.getStringTopic("notification").publish();

    /**
     * Switches the active tab on the Elastic Dashboard.
     */
    public static void selectTab(String tabName) {
        selectedTabPub.set(tabName);
    }

    /**
     * Sends a notification banner to the Elastic Dashboard.
     */
    public static void sendNotification(Notification notification) {
        notificationPub.set(notification.toJson());
    }

    public static class Notification {
        public enum Level {
            INFO, WARNING, ERROR
        }

        private final String title;
        private final String message;
        private final Level level;

        private Notification(String title, String message, Level level) {
            this.title = title;
            this.message = message;
            this.level = level;
        }

        public String toJson() {
            return String.format(
                    "{\"title\":\"%s\",\"description\":\"%s\",\"level\":\"%s\"}",
                    title, message, level.name()
            );
        }

        public static Builder builder() {
            return new Builder();
        }

        public static class Builder {
            private String title = "";
            private String message = "";
            private Level level = Level.INFO;

            public Builder withTitle(String title) {
                this.title = title;
                return this;
            }

            public Builder withMessage(String message) {
                this.message = message;
                return this;
            }

            public Builder withLevel(Level level) {
                this.level = level;
                return this;
            }

            public Notification build() {
                return new Notification(title, message, level);
            }
        }
    }
}