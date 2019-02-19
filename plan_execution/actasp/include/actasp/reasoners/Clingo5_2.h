#pragma once

#include <actasp/FilteringQueryGenerator.h>
#include <boost/filesystem.hpp>
#include <actasp/filesystem_utils.h>

namespace Clingo {
class Model;
}

namespace actasp {

static std::set<std::string> standard_program_names = {"base", "step", "check", "knowledge"};

class AspProgram;

struct Clingo5_2 : public FilteringQueryGenerator {

  explicit Clingo5_2(
      const std::vector<std::string> &domain_files
  ) noexcept;

  std::vector<Plan> minimalPlanQuery(const std::vector<AspRule> &goalRules,
                                     unsigned int max_plan_length,
                                     unsigned int answerset_number,
                                     const std::vector<AspFact> *knowledge) const noexcept override;

  std::vector<Plan> lengthRangePlanQuery(const std::vector<AspRule> &goal,
                                         unsigned int min_plan_length,
                                         unsigned int max_plan_length,
                                         unsigned int answerset_number,
                                         const std::vector<AspFact> *knowledge
  ) const noexcept override;

  Plan optimalPlanQuery(const std::vector<AspRule> &goal,
                        unsigned int max_plan_length,
                        unsigned int answerset_number,
                        const std::vector<AspFact> *knowledge
  ) const noexcept override;

  std::vector<Plan> monitorQuery(const std::vector<AspRule> &goal,
                                 const Plan &plan, const std::vector<AspFact> *knowledge) const noexcept override;

  AnswerSet currentStateQuery(const std::vector<AspRule> &query) const noexcept override;

  std::vector<Plan> filteringQuery(const AnswerSet &currentState, const Plan &plan, const std::vector<AspRule> &goal,
                                   const std::vector<AspFact> *knowledge) const noexcept override;

  std::vector<AnswerSet> genericQuery(const std::vector<AspProgram> &programs,
                                      unsigned int min_plan_length,
                                      unsigned int max_plan_length,
                                      unsigned int max_num_plans,
                                      bool shortest_only,
                                      std::vector<AspFact> *knowledge) const noexcept;

  std::vector<AnswerSet> genericQuery(const std::vector<AspProgram> &programs,
                                      unsigned int min_plan_length,
                                      unsigned int max_plan_length,
                                      unsigned int max_num_plans,
                                      bool shortest_only) const noexcept override;


  std::set<std::string> get_all_actions() override {
    return action_names;
  };


  typedef std::function<void (const Clingo::Model&)> ModelCallback;
private:

  void
  makeQuery(const std::vector<AspProgram> &programs, ModelCallback model_cb, unsigned int min_plan_length, unsigned int max_plan_length,
            unsigned int max_num_plans, bool shortest_only=true) const noexcept;

  static std::string program_to_string(const AspProgram &program);

  template <class T>
  void log_query(const std::vector<AspProgram> &programs, std::vector<T> results) const {
    using namespace boost::filesystem;
    using namespace std;
    path queryDir = get_query_directory(domain_files, {});
    auto queryDirFiles = populate_directory(queryDir, domain_files);

    set<string> non_standard_programs_names;
    for (const auto &program: programs) {
      // Use the nice name provided by the user to make the file names more helpful
      const path part_path = (queryDir / program.user_facing_name).string() + ".asp";
      // We'll open this file creating it it if it doesn't exist and appending to it otherwise
      std::ofstream query_file(part_path.string(), std::ofstream::app);
      query_file << program_to_string(program);
      query_file.close();
      non_standard_programs_names.insert(program.user_facing_name);
    }

    string filename = "generic_query_";
    set<string> custom_user_program_names;
    set_difference(non_standard_programs_names.begin(), non_standard_programs_names.end(), standard_program_names.begin(), standard_program_names.end(), inserter(custom_user_program_names, custom_user_program_names.end()));
    if (!custom_user_program_names.empty()) {
      stringstream nice_filename;
      for (const auto &name: custom_user_program_names) {
        nice_filename << name << "_";
      }
      filename = nice_filename.str();
    }

    const path result_file_path = (queryDir / filename).string() + "output.txt";
    std::ofstream result_file(result_file_path.string(), std::ofstream::app);

    int i = 0;
    for (const auto &set: results) {
      result_file << "% Model " << i << endl;
      result_file << set.to_string() << endl << endl << endl;
      i++;
    }
  }


  std::string incremental_var;
  std::set<std::string> action_names;
  std::set<std::string> fluent_names;
  std::vector<std::string> domain_files;
public:
  bool log_queries;

};

}


