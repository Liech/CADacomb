#include "IProgressBar.h"

namespace CADacomb
{
    IProgressBar::IProgressBar() {}

    IProgressBar::~IProgressBar() {}
}

#ifdef ISTESTPROJECT
#include <catch2/catch_test_macros.hpp>

namespace CADacomb
{
    class TestProgressBar : public IProgressBar
    {
      public:
        TestProgressBar();
        virtual ~TestProgressBar();

        void        setProgress(float progressInput, const std::string& messageInput) override;
        float       getProgress() const override;
        std::string getMessage() const override;

      private:
        float       progress = 0.0f;
        std::string message  = "";
    };

    TestProgressBar::TestProgressBar() {}

    TestProgressBar::~TestProgressBar() {}

    void TestProgressBar::setProgress(float progressInput, const std::string& messageInput)
    {
        progress = progressInput;
        message  = messageInput;
    }

    float TestProgressBar::getProgress() const
    {
        return progress;
    }

    std::string TestProgressBar::getMessage() const
    {
        return message;
    }

    TEST_CASE("IProgressBar/SetAndGetProgress")
    {
        TestProgressBar bar;
        bar.setProgress(0.5f, "Halfway");
        CHECK(bar.getProgress() == 0.5f);
        CHECK(bar.getMessage() == "Halfway");
    }
}
#endif
