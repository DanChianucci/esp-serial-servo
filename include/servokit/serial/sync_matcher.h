//------------------------------------------------------------------------
// Filename  :  sync_matcher.h
// Project   :  servokit
//
// Description:
//    SyncMatcher: finds a frame sync pattern in a byte stream. Header-only,
//    no I/O, never blocks, no heap, no platform dependencies. Not
//    thread-safe: use one matcher per stream.
//------------------------------------------------------------------------

#ifndef SERVOKIT_SYNC_MATCHER__H
#define SERVOKIT_SYNC_MATCHER__H

#include <cstddef>
#include <cstdint>
#include <span>

namespace servokit {

/** Streaming matcher that detects a byte pattern in a byte stream, one byte
 *  at a time (Knuth-Morris-Pratt). It finds the pattern wherever it starts,
 *  including after partial matches that overlap the true start, and uses
 *  memory bounded by MAX_PATTERN. It has no platform dependencies.
 *
 *  The matcher refers to the pattern passed to reset(); that storage must stay
 *  valid and unchanged while the matcher is used.
 */
class SyncMatcher {
 public:
  /** Longest supported pattern, in bytes. */
  static constexpr size_t MAX_PATTERN = 16;

  /** Sets the pattern and restarts matching.
   *  An empty pattern is matched from the start (complete() is true).
   *  @return true on success.
   *  @return false if the pattern is longer than MAX_PATTERN; the matcher is
   *          then empty until a valid reset().
   */
  bool reset(std::span<const uint8_t> pattern) {
    m_pattern = {};
    m_state = 0;
    if (pattern.size() > MAX_PATTERN) return false;
    m_pattern = pattern;
    // Build the KMP failure table: m_fail[i] is the length of the longest
    // proper prefix of pattern[0..i] that is also its suffix. On a mismatch
    // the matcher falls back to that length instead of restarting at 0, which
    // is what finds a pattern that begins inside a partial match.
    size_t k = 0;
    for (size_t i = 1; i < pattern.size(); ++i) {
      while (k > 0 && pattern[i] != pattern[k]) k = m_fail[k - 1];
      if (pattern[i] == pattern[k]) ++k;
      m_fail[i] = static_cast<uint8_t>(k);
    }
    return true;
  }

  /** Restarts matching with the current pattern. */
  void restart() { m_state = 0; }

  /** Consumes one byte of the stream.
   *  After a complete match, the next byte continues the search for a further
   *  (possibly overlapping) match; call restart() to discard that overlap.
   *  @return true when the byte completes the pattern.
   */
  bool feed(uint8_t byte) {
    const size_t n = m_pattern.size();
    if (n == 0) return true;
    // After a full match, continue from the longest border so overlapping
    // matches (for example AA in AAA) are still found.
    if (m_state == n) m_state = m_fail[n - 1];
    while (m_state > 0 && byte != m_pattern[m_state]) m_state = m_fail[m_state - 1];
    if (byte == m_pattern[m_state]) ++m_state;
    return m_state == n;
  }

  /** @return true when the bytes consumed since the last restart() or reset()
   *          end with the pattern, or the pattern is empty. */
  bool complete() const { return m_state == m_pattern.size(); }

 private:
  std::span<const uint8_t> m_pattern{};
  uint8_t m_fail[MAX_PATTERN] = {};  //!< Longest proper border of each pattern prefix.
  size_t m_state = 0;                //!< Pattern bytes currently matched.
};

}  // namespace servokit

#endif  // SERVOKIT_SYNC_MATCHER__H
