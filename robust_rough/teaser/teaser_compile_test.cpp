#include <teaser/teaser_align.hpp>
#include <teaser/teaser_respondence.hpp>

#include <iostream>

int main() {
  teaser::TeaserAlign aligner;
  aligner.setMethod(teaser::TeaserAlign::Method::FGNC_TLS);
  aligner.setDSampleRadius(0.1);

  teaser::TeaserRespondence respondence;
  (void)respondence;

  std::cout << "teaser compile ok\n";
  return 0;
}
