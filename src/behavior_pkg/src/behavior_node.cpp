enum class State {SEARCH, TRACK};

enum class Event {TARGET_CONFIRMED, TARGET_LOST, NO_EVENT};


State next_state(State current, Event e)
{
    switch (current) {
        case State::SEARCH:
            if (e == Event::TARGET_CONFIRMED) return State::TRACK;
            return current;

        case State::TRACK:
            if (e == Event::TARGET_LOST) return State::SEARCH;
            return current;
        }

    return current; // unreachable but keeps compiler quiet.
}


class FrameFilter
{
public:
    FrameFilter(int frames_to_confirm, int frames_to_drop)
     : n_(frames_to_confirm), m_(frames_to_drop) 
    {

    }

    Event produce_event(bool is_valid_frame)
    {
        if (is_valid_frame) {
            ++confirm_counter_;
            drop_counter_ = 0;
        } else {
            ++drop_counter_;
            confirm_counter_ = 0;
        }

        if (confirm_counter_ >= n_) {
            return Event::TARGET_CONFIRMED;
        } else if (drop_counter_ >= m_) {
            return Event::TARGET_LOST;
        } 

        return Event::NO_EVENT;

    }
private:

    int n_; // Amount of frames needed to confirm a state
    int m_; // Amount of frames needed to release a state

    int confirm_counter_ { 0 };
    int drop_counter_ { 0 };

};