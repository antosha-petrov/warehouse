using System.Windows.Input;

namespace WarehouseApp.ViewModel;

public class CardViewModel : BindableObject
{
    private int _quantity;

    public string ImageSource { get; set; }
    public string Title { get; set; }

    public int Quantity
    {
        get => _quantity;
        set
        {
            _quantity = value;
            OnPropertyChanged();
        }
    }

    public ICommand IncrementCommand { get; }
    public ICommand DecrementCommand { get; }

    public CardViewModel(string title, string imageSource)
    {
        Title = title;
        ImageSource = imageSource;
        Quantity = 0;

        IncrementCommand = new Command(() => Quantity++);
        DecrementCommand = new Command(() =>
        {
            if (Quantity > 0)
                Quantity--;
        });
    }
}
