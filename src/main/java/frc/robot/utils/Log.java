
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.ConsoleHandler;
import java.util.logging.FileHandler;
import java.util.logging.Filter;
import java.util.logging.Formatter;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

/**
 * Generic logger class that will print statements to the Console or save to a
 * file. Allows usage of multiple parameters and supports filter to filter out
 * log messages being printed.
 */

public class Log extends SecurityManager {
    private final static boolean NOLOG = true;
    private Logger logger;
    private static Object _instance_lock = new Object();
    private volatile static Log _instance = null;
    private static String sfilterByClassName;

    private Log() {
        LogManager.getLogManager().reset();
        logger = Logger.getLogger(this.getClass().getCanonicalName());
        Handler lh = new ConsoleHandler();
        lh.setFormatter(new LogFormatter());

        var lhs = logger.getHandlers();
        logger.addHandler(lh);
    }

    public static Log instance() {
        if (_instance == null) {
            synchronized (_instance_lock) {
                if (_instance == null) {
                    _instance = new Log();
                }
            }
        }

        return _instance;
    }

    public static String variableArgumentsHandler(String label, Object... args) {
        StringBuilder sb = new StringBuilder();
        sb.append(label).append(": ");

        for (Object object : args) {
            try {
                if (object != null)
                    sb.append(object.toString()).append(" ");
            } catch (Exception e) {
                // Ignore Exception
            }
        }

        return sb.toString();
    }

    public static void setFilter(Level level) {
        _instance.logger.setLevel(level);
    }

    /**
     * Log a severe error messages. The first argument is a label and the rest of
     * the arguments are concatenated to it.
     * 
     * @param label
     * @param args
     * @return
     */
    public static Log severe(String label, Object... args) {
        Log log = Log.instance();

        severe(variableArgumentsHandler(label, args));
        return log;
    }

    /**
     * Log a severe error message.
     * 
     * @param message
     * @return
     */
    public static Log severe(String message) {
        Log log = Log.instance();

        log.logger.severe("[" + getCallerClassName() + "] - " + message);

        return log;
    }

    /**
     * Logs severe error messages with Java's String.format. Use %% to represent a %
     * sign.
     * 
     * Example: int iVal = 10; double dVal = 11.1; String sVal = "Test"
     * Log.severeF("Example message for %s - iVal=%d, dVal=%f, sVal=%s", iVal, dVal,
     * sVal);
     * 
     * @param formatStr - format string with %d for
     * @param args      - all the placeholder values
     * @return Log for chaining purposes
     */
    public static Log severeF(String formatStr, Object... args) {
        Log log = Log.instance();

        try {

            severe(String.format(formatStr, args));
        } catch (Exception e) {
            // Ignore errors
        }

        return log;
    }

    /**
     * Log an information message. The first argument is a label and the rest of the
     * arguments are concatenated to it.
     * 
     * @param label
     * @param args
     * @return
     */
    public static Log info(String label, Object... args) {
        Log log = Log.instance();

        info(variableArgumentsHandler(label, args));
        return log;
    }

    /**
     * Logs informational messages with Java's String.format. Use %% to represent a
     * % sign.
     * 
     * Example: int iVal = 10; double dVal = 11.1; String sVal = "Test"
     * Log.infoF("Example message for %s - iVal=%d, dVal=%f, sVal=%s", iVal, dVal,
     * sVal);
     * 
     * @param formatStr - format string with %d for
     * @param args      - all the placeholder values
     * @return Log for chaining purposes
     */
    public static Log infoF(String formatStr, Object... args) {
        if (!NOLOG) {
            Log log = Log.instance();

            try {
                info(String.format(formatStr, args));
            } catch (Exception e) {
                // ignore error
            }
            return log;
        }

        return null;
    }

    /**
     * Log an informational message.
     * 
     * @param message
     * @return
     */
    public static Log info(String message) {
        Log log = Log.instance();

        log.logger.info("[" + getCallerClassName() + "] - " + message);
        // log.logger.info(message);
        return log;
    }

    public static String getCallerClassName() {

        /*
         * StackTraceElement[] stElements = Thread.currentThread().getStackTrace();
         * String callerClassName = null; for (int i = 1; i < stElements.length; i++) {
         * StackTraceElement ste = stElements[i]; if
         * (!ste.getClassName().equals(Log.class.getName()) &&
         * ste.getClassName().indexOf("java.lang.Thread") != 0) { if (callerClassName ==
         * null) { callerClassName = ste.getClassName(); } else if
         * (!callerClassName.equals(ste.getClassName())) { return ste.getClassName(); }
         * } }
         */
        String callerClassName = Log.instance().getClassContext()[1].getName();
        // int i=0;
        for (Class<?> lClass : Log.instance().getClassContext()) {
            // System.out.println("Class: [" + i++ + "] " + lClass.getName());

            if (!lClass.getName().equals(Log.class.getName())) {
                callerClassName = lClass.getName();
                break;
            }
        }
        return callerClassName;
    }

    public void setFormatter() {
        for (Handler handler : logger.getHandlers()) {
            handler.setFormatter(new LogFormatter());
        }
    }

    /**
     * Save the log to a file name. In the robot, save to /home/lvuser/log.txt. FTP
     * to robot to get to this file. When you FTP, log in with lvuser username and
     * no password.
     * 
     * The file is overwritten everytime. Provide a unique name if you need to
     * retain it.
     * 
     * @param fileName
     */
    public static void saveToFile(String fileName) {
        FileHandler fh;
        Log log = Log.instance();

        try {
            // This block configure the logger with handler and formatter
            fh = new FileHandler(fileName);
            log.logger.addHandler(fh);
            // SimpleFormatter formatter = new SimpleFormatter();
            log.setFormatter();

        } catch (SecurityException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        Log.info("Testing");
    }

    public class LogFormatter extends Formatter {

        @Override
        public String format(LogRecord record) {
            DateFormat dateFormat = new SimpleDateFormat("mm/dd HH:mm:ss.SSS");

            String now = dateFormat.format(new Date());

            return String.format("%s %s::%s\n", now, record.getLevel().getName().substring(0, 1), record.getMessage());
            // return "(" + now + " " + record.getLevel().getName().substring(0, 1) + "::" +
            // record.getMessage() + "\n";
        }
    }

    public class CustomFilter implements Filter {
        public boolean isLoggable(LogRecord record) {

            return (record.getMessage().startsWith(sfilterByClassName + ")"));
        }
    }
}
