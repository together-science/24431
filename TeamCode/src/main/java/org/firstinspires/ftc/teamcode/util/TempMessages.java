package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.stream.Collectors;

public class TempMessages {
    private static class TempMessageEntry {
        public TempMessageEntry (String msg, long expires) {
            this.msg = msg;
            this.expires = expires;
        }
        String msg;
        long expires;
    }

    private ArrayList<TempMessageEntry> msgs = new ArrayList<>();

    public void add(String msg, int ms) {
        msgs.add(new TempMessageEntry(msg, System.currentTimeMillis() + ms));
    }

    public void add(String msg) {
        add(msg, 5000);
    }

    public String[] getAllCurrent() {
        long now = System.currentTimeMillis();
        msgs = msgs.stream()
                .filter((TempMessageEntry e) -> e.expires > now)
                .collect(Collectors.toCollection(ArrayList::new));
        String[] result = msgs.stream()
                .map((TempMessageEntry e)-> e.msg)
                .toArray(String[]::new);
        return result;
    }
}
