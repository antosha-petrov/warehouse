using System.Windows.Input;

namespace WarehouseApp.ViewModel
{
    public class ThankForOrderViewModel
    {
        public ICommand NavigateToMain { get; }

        public ThankForOrderViewModel()
        {
            NavigateToMain = new Command(() => Shell.Current.GoToAsync("//Main"));
        }
    }
}
