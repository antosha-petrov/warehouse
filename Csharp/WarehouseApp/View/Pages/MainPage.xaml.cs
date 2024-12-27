using System.Runtime.CompilerServices;
using WarehouseApp.ViewModel;

namespace WarehouseApp
{
    public partial class MainPage : ContentPage
    {
        public MainPage()
        {
            InitializeComponent();
            BindingContext = new MainPageViewModel();
        }

        private async void OnOrderButtonClicked(object sender, EventArgs e)
        {
            await Shell.Current.GoToAsync("//Order");
        }

    }

}
