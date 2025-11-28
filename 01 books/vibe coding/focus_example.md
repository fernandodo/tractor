# 🎯 MAIN FOCUS (Only ONE allowed)

**Today's primary task:**\
\> Write and validate the UART frame parser (header → length → payload →
CRC)

🚫 **Do NOT do today:**\
- No refactoring\
- No future redesign\
- No optimization\
- No exploring other modules\
- No "just checking something"

Stay on the main line.

------------------------------------------------------------------------

# 📍 CURRENT WORKING CONTEXT (Where I stopped last time)

-   Finished implementing `parse_header()`, but CRC check is not
    integrated yet.\
-   Need to verify `sequence_number` wrap-around behavior.\
-   Debug output currently prints raw bytes but lacks payload length.\
-   Encountered inconsistent frame type field (needs clarification).

------------------------------------------------------------------------

# ⏭ NEXT ACTIONS (Each should take 5--15 minutes)

1.  Implement `crc_check_header()` and plug it into the parser.
2.  Add payload-length logging inside `parse_payload()`.
3.  Create 3 sample test frames and run through the parser.
4.  Verify behavior when sequence number exceeds 255 → wraps to 0.

------------------------------------------------------------------------

# 🧱 BLOCKERS / OPEN QUESTIONS

-   What is the *maximum* payload size? Need confirmation from the
    design doc.\
-   Should payload decoding live in the **transport layer** or
    **application layer**?\
-   Is segmentation handled here or by the upper layer?

------------------------------------------------------------------------

# 🧠 DESIGN NOTES / BRAIN CACHE (Do NOT work on these today)

-   Parser state machine needs an explicit `ERROR` state.\
-   Might need a ring buffer for future segmentation support.\
-   Timeout handling could be centralized in the event loop.\
-   Consider abstracting CRC functions for future protocol versions.

------------------------------------------------------------------------

# 🔄 DAILY CHECKLIST

-   [ ] Header CRC integrated\
-   [ ] Payload length logging added\
-   [ ] Sample frames tested\
-   [ ] Sequence-number wrap-around validated

------------------------------------------------------------------------

# 📝 LINKS / REFERENCES

-   Project protocol spec → `docs/protocol_v2.md`\
-   Test logs → `logs/uart_parser_tests/`\
-   Parser code → `src/transport/uart/parser.c`

------------------------------------------------------------------------

# 🧩 SESSION LOG

Example:

    - CRC mismatch traced to byte-order issue, fixed.
    - Need to clean up debug logging after validation.
    - Consider adding a small test harness later.

------------------------------------------------------------------------

# 🏁 END-OF-DAY WRAP-UP

Example:

    Done: Header CRC implemented and tested.
    Stopped: While creating sample test frames.
    Tomorrow: Start by testing sequence wrap-around.
